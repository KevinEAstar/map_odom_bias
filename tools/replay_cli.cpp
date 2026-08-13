/**
 * @file replay_cli.cpp
 * @brief 离线确定性回放判卷工具: 从 CSV (extract_bag_inputs.py 产出) 重放
 *        观测/odom/reset 事件流喂 BiasEstimator, 输出账本序列。
 *
 * 用途 (台架回归, 设计文档 v1 4.5): 同一飞行 bag 输入下双跑对照 ——
 *   --metric=param  旧度量 (变换参数空间, 评估点不传 = 原点退化)
 *   --metric=body   新度量 (③修法机体点残差, 门控传 p_ob / tick 传最新样本)
 * 每帧观测同时记录两种度量的 err 值 (门控行为由模式决定, err 双记只为分析)。
 *
 * 事件时序: 三路事件按 t_bag (bag 写入戳≈到达刻) 归并; tick 以 50Hz 栅格
 * 穿插在事件流中 (栅格钟 = t_bag 轴)。odom push 与 obs 配对用 t_hdr
 * (header 戳), 与在线节点行为一致。reset 事件复刻 Px4ResetSource 的
 * D 合成 (NED→ENU, 先位置后航向)。
 *
 * 用法: replay_cli <inputs_dir> <out_dir> --metric=param|body
 * 输出: <out_dir>/obs_ledger.csv (每观测一行) + tick_ledger.csv (1Hz 快照)
 *       + summary.txt (终值账)
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "map_odom_bias/core/bias_estimator.hpp"
#include "map_odom_bias/core/obs_intake.hpp"
#include "map_odom_bias/core/odom_buffer.hpp"
#include "map_odom_bias/core/pose_math.hpp"
#include "map_odom_bias/core/time_types.hpp"

namespace pm = map_odom_bias::pose_math;
using map_odom_bias::BiasEstimator;
using map_odom_bias::BiasEstimatorParams;
using map_odom_bias::BiasState;
using map_odom_bias::OdomBuffer;
using map_odom_bias::HostTime;
using map_odom_bias::OdomSample;
using map_odom_bias::SampleTime;

namespace
{

struct ObsRow
{
    double t_bag, t_hdr;
    pm::Pose pose;              // pose 模式: map 系机体位姿 (需与 odom 配对)
    pm::Transform4D tf;         // tf 模式: 现成 T_map_odom 观测变换
    bool is_tf{false};
};

struct OdomRow
{
    double t_bag, t_hdr;
    pm::Pose pose;
};

struct ResetRow
{
    double t_bag;
    int xy, z, heading;
    double dxy0, dxy1, dz, dheading;    // NED delta
    double x, y, z_pos;                 // NED 当前位置
};

// NED→ENU (坐标系铁律, 与 Px4ResetSource 同构)
std::array<double, 3> ned_to_enu(double xn, double yn, double zn)
{
    return {{yn, xn, -zn}};
}

std::vector<std::vector<double>> read_csv(const std::string & path)
{
    std::vector<std::vector<double>> rows;
    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "无法打开 %s\n", path.c_str());
        std::exit(1);
    }
    std::string line;
    std::getline(f, line);    // 表头
    while (std::getline(f, line)) {
        if (line.empty()) {
            continue;
        }
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::atof(cell.c_str()));
        }
        rows.push_back(row);
    }
    return rows;
}

pm::Pose pose_from(const std::vector<double> & r, std::size_t off)
{
    pm::Pose p;
    p.p = {{r[off], r[off + 1], r[off + 2]}};
    p.q.w = r[off + 3];
    p.q.x = r[off + 4];
    p.q.y = r[off + 5];
    p.q.z = r[off + 6];
    return p;
}

double norm3d(double x, double y, double z)
{
    return std::sqrt(x * x + y * y + z * z);
}

}  // namespace

int main(int argc, char ** argv)
{
    if (argc < 4) {
        std::fprintf(stderr,
            "用法: replay_cli <inputs_dir> <out_dir> --metric=param|body "
            "[--soft-trans=X --soft-yaw=Y --band=Q --median=N]\n");
        return 1;
    }
    const std::string in_dir = argv[1];
    const std::string out_dir = argv[2];
    const bool body_metric = std::strcmp(argv[3], "--metric=body") == 0;

    // ---- v2 质量链开关 (离线开带对照; 缺省全关 = v1 行为逐位保持) ----
    //   --soft-trans=Δ --soft-yaw=Δψ : 三带正常带上界 (缺省跟随 gate)
    //   --band=q                     : 降权带质量因子 (缺省 1/3)
    //   --median=N                   : 观测中值窗 (缺省 1 = 关)
    double soft_trans = -1.0, soft_yaw = -1.0, band_q = 1.0 / 3.0;
    int median_window = 1;
    for (int i = 4; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--soft-trans=", 0) == 0) {
            soft_trans = std::atof(a.c_str() + 13);
        } else if (a.rfind("--soft-yaw=", 0) == 0) {
            soft_yaw = std::atof(a.c_str() + 11);
        } else if (a.rfind("--band=", 0) == 0) {
            band_q = std::atof(a.c_str() + 7);
        } else if (a.rfind("--median=", 0) == 0) {
            median_window = std::atoi(a.c_str() + 9);
        } else {
            std::fprintf(stderr, "未知参数: %s\n", a.c_str());
            return 1;
        }
    }

    // ---- 装载三路输入 (obs_tf.csv 存在时优先: 现成观测变换模式) ----
    std::vector<ObsRow> obs;
    {
        std::ifstream probe(in_dir + "/obs_tf.csv");
        if (probe.good()) {
            for (const auto & r : read_csv(in_dir + "/obs_tf.csv")) {
                ObsRow row;
                row.t_bag = r[0];
                row.t_hdr = r[1];
                row.tf.x = r[2];
                row.tf.y = r[3];
                row.tf.z = r[4];
                row.tf.yaw = r[5];
                row.is_tf = true;
                obs.push_back(row);
            }
        } else {
            for (const auto & r : read_csv(in_dir + "/obs.csv")) {
                ObsRow row;
                row.t_bag = r[0];
                row.t_hdr = r[1];
                row.pose = pose_from(r, 2);
                obs.push_back(row);
            }
        }
    }
    std::vector<OdomRow> odom;
    for (const auto & r : read_csv(in_dir + "/odom.csv")) {
        odom.push_back({r[0], r[1], pose_from(r, 2)});
    }
    std::vector<ResetRow> resets;
    for (const auto & r : read_csv(in_dir + "/reset_events.csv")) {
        resets.push_back({r[0], static_cast<int>(r[1]), static_cast<int>(r[2]),
                          static_cast<int>(r[3]), r[4], r[5], r[6], r[7],
                          r[8], r[9], r[10]});
    }
    std::fprintf(stderr, "输入: obs=%zu odom=%zu reset=%zu metric=%s\n",
                 obs.size(), odom.size(), resets.size(),
                 body_metric ? "body" : "param");

    // ---- 核心件 (参数 = 代码默认 = 08-06 板端配置 + 质量链开关) ----
    OdomBuffer buffer(4.0, 0.05);
    BiasEstimatorParams bp;
    bp.gate_soft_trans_threshold = soft_trans;
    bp.gate_soft_yaw_threshold = soft_yaw;
    bp.band_quality = band_q;
    BiasEstimator est(bp);
    // 中值窗与在线同件同位 (intake 链, 门控前); SOURCE_JUMP/reset 后
    // 洗窗对齐节点行为 (jump_count 变化检测 / apply_reset 分支)
    map_odom_bias::MedianWindow median(median_window);
    std::fprintf(stderr,
                 "质量链: soft=%.3f/%.3f band=%.3f median=%d\n",
                 bp.gate_soft_trans_threshold, bp.gate_soft_yaw_threshold,
                 bp.band_quality, median_window);
    static constexpr double kTickDt = 0.02;             // 50 Hz
    static constexpr double kSettleDuration = 0.05;     // reset 静默窗

    std::ofstream f_obs(out_dir + "/obs_ledger.csv");
    f_obs << std::setprecision(15);    // 绝对时间秒 (1.8e9) 需 15 位有效数字
    f_obs << "t_bag,t_hdr,query_ok,state,gate_reject,jump_count,"
             "raw_x,raw_y,raw_z,raw_yaw,err_param_trans,err_body_trans,"
             "err_yaw,p_ob_norm,q_eff\n";
    std::ofstream f_tick(out_dir + "/tick_ledger.csv");
    f_tick << std::setprecision(15);
    f_tick << "t,state,ctrl_x,ctrl_y,ctrl_z,ctrl_yaw,"
              "cmd_x,cmd_y,cmd_z,cmd_yaw,div_param,div_body\n";

    // ---- 事件归并主循环 (odom/obs/reset 按 t_bag; tick 栅格穿插) ----
    std::size_t io = 0, ib = 0, ir = 0;
    double t_tick = odom.empty() ? 0.0 : odom.front().t_bag;
    double t_last_tick_log = -1e18;

    auto eval_pts = [&]() {
        std::vector<std::array<double, 3>> pts;
        if (body_metric && !buffer.empty()) {
            pts.push_back(buffer.newest_pose().p);
        }
        return pts;
    };

    auto do_tick = [&](double t) {
        est.tick(HostTime{t}, eval_pts());
        if (t - t_last_tick_log >= 1.0) {    // 1 Hz 快照
            t_last_tick_log = t;
            std::vector<std::array<double, 3>> pts;
            if (!buffer.empty()) {
                pts.push_back(buffer.newest_pose().p);
            }
            const auto & c = est.ctrl();
            const auto & m = est.cmd();
            f_tick << t << ',' << static_cast<int>(est.state()) << ','
                   << c.x << ',' << c.y << ',' << c.z << ',' << c.yaw << ','
                   << m.x << ',' << m.y << ',' << m.z << ',' << m.yaw << ','
                   << est.divergence_trans()
                   << ',' << est.divergence_trans(pts) << '\n';
        }
    };

    while (io < odom.size() || ib < obs.size() || ir < resets.size()) {
        const double t_o = io < odom.size() ? odom[io].t_bag : 1e18;
        const double t_b = ib < obs.size() ? obs[ib].t_bag : 1e18;
        const double t_r = ir < resets.size() ? resets[ir].t_bag : 1e18;
        const double t_next = std::min(t_o, std::min(t_b, t_r));

        // tick 栅格追到下一事件前
        while (t_tick + kTickDt <= t_next) {
            t_tick += kTickDt;
            do_tick(t_tick);
        }

        if (t_o <= t_b && t_o <= t_r) {
            OdomSample s;
            s.t = SampleTime{odom[io].t_hdr};
            s.pose = odom[io].pose;
            buffer.push(s);
            ++io;
        } else if (t_b <= t_r) {
            const ObsRow & row = obs[ib];
            pm::Pose odom_base;
            const bool ok =
                buffer.query(SampleTime{row.t_hdr}, &odom_base) ==
                OdomBuffer::QueryResult::OK;
            if (ok) {
                // tf 模式: 观测变换现成 (p_ob 仍需配对提供评估点);
                // pose 模式: 由位姿对构造 4DoF 偏差观测
                pm::Transform4D t_obs = row.is_tf
                    ? row.tf
                    : pm::bias_observation(row.pose, odom_base);
                // 中值窗 (在线同件同位: 门控前观测流)
                if (median_window > 1) {
                    map_odom_bias::GlobalPoseObservation g;
                    g.t_sample = SampleTime{row.t_hdr};
                    g.t_arrival = HostTime{row.t_bag};
                    g.T_obs = t_obs;
                    g.p_ob = odom_base.p;
                    median.process(g, est);
                    t_obs = g.T_obs;
                }
                // 双度量 err 只为分析记录 (门控行为由模式决定)
                const pm::Transform4D & raw = est.raw();
                const pm::TransformError e_param = pm::transform_error(
                    t_obs, raw, std::vector<std::array<double, 3>>{});
                const pm::TransformError e_body = pm::transform_error(
                    t_obs, raw, std::vector<std::array<double, 3>>{odom_base.p});
                // 采样刻 = header 戳, 到达刻 = bag 写入戳 (回放保真)
                const uint32_t jumps_before = est.jump_count();
                if (body_metric) {
                    est.add_observation(t_obs, SampleTime{row.t_hdr},
                                        HostTime{row.t_bag}, odom_base.p);
                } else {
                    est.add_observation(t_obs, SampleTime{row.t_hdr},
                                        HostTime{row.t_bag});
                }
                if (est.jump_count() != jumps_before) {
                    median.reset();    // SOURCE_JUMP 后洗窗 (对齐节点 reset_all)
                }
                f_obs << row.t_bag << ',' << row.t_hdr << ",1,"
                      << static_cast<int>(est.state()) << ','
                      << est.gate_reject_count() << ',' << est.jump_count() << ','
                      << est.raw().x << ',' << est.raw().y << ','
                      << est.raw().z << ',' << est.raw().yaw << ','
                      << e_param.trans << ',' << e_body.trans << ','
                      << e_param.yaw << ','
                      << norm3d(odom_base.p[0], odom_base.p[1], odom_base.p[2])
                      << ',' << est.last_obs_quality()
                      << '\n';
            } else {
                f_obs << row.t_bag << ',' << row.t_hdr << ",0,"
                      << static_cast<int>(est.state()) << ','
                      << est.gate_reject_count() << ',' << est.jump_count()
                      << ",,,,,,,,,\n";
            }
            ++ib;
        } else {
            const ResetRow & r = resets[ir];
            pm::Transform4D d_total;
            if (r.xy || r.z) {
                const double dx = r.xy ? r.dxy0 : 0.0;
                const double dy = r.xy ? r.dxy1 : 0.0;
                const double dz = r.z ? r.dz : 0.0;
                const auto dp = ned_to_enu(dx, dy, dz);
                d_total.x = dp[0];
                d_total.y = dp[1];
                d_total.z = dp[2];
            }
            if (r.heading) {
                const double dpsi = -r.dheading;
                const auto p_ob = ned_to_enu(r.x, r.y, r.z_pos);
                d_total = pm::compose(
                    pm::make_heading_reset_delta(dpsi, p_ob), d_total);
            }
            est.apply_reset(d_total);
            median.reset();    // 窗内历史属旧坐标系 (对齐节点 reset_all)
            buffer.clear_and_settle(
                SampleTime{r.t_bag + kSettleDuration});
            ++ir;
        }
    }

    // ---- 终值账 ----
    std::ofstream f_sum(out_dir + "/summary.txt");
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "metric=%s\nstate=%d\ngate_reject=%u\njump_count=%u\n"
        "invalid_obs=%u\nreset_events=%u\nobs_too_old=%u\nobs_too_new=%u\n"
        "last_jump_trans=%.3f\nlast_jump_yaw=%.3f\n",
        body_metric ? "body" : "param", static_cast<int>(est.state()),
        est.gate_reject_count(), est.jump_count(), est.invalid_obs_count(),
        est.reset_event_count(), buffer.too_old_count(), buffer.too_new_count(),
        est.last_jump_trans(), est.last_jump_yaw());
    f_sum << buf;
    std::fprintf(stderr, "%s", buf);
    return 0;
}

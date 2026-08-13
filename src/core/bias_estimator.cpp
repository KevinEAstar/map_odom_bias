/**
 * @file bias_estimator.cpp
 * @brief BiasEstimator 实现: 状态机 / 门控 / 慢吸收 / reset 补偿
 */

#include "map_odom_bias/core/bias_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace map_odom_bias
{

namespace pm = pose_math;

namespace
{
double clamp01(double q)
{
    return std::max(0.0, std::min(1.0, q));
}
}  // namespace

BiasEstimator::BiasEstimator(const BiasEstimatorParams & params)
: params_(params)
{
    // v2 质量链参数解析 (设计文档 v1 4.4): soft 哨兵 <0 → 跟随 gate
    // (降权带宽零 = 旧行为); Δ≤L 加载期硬断言 —— MRS rtk 配置把
    // limit==max 配成钳位层静默架空的反例对治, 配置错误 fail-fast
    if (params_.gate_soft_trans_threshold < 0.0) {
        params_.gate_soft_trans_threshold = params_.gate_trans_threshold;
    }
    if (params_.gate_soft_yaw_threshold < 0.0) {
        params_.gate_soft_yaw_threshold = params_.gate_yaw_threshold;
    }
    if (params_.gate_soft_trans_threshold > params_.gate_trans_threshold ||
        params_.gate_soft_yaw_threshold > params_.gate_yaw_threshold) {
        throw std::invalid_argument(
            "BiasEstimator: gate_soft_* 必须 ≤ gate_* (三带 Δ≤L)");
    }
    if (!(params_.band_quality > 0.0 && params_.band_quality <= 1.0)) {
        throw std::invalid_argument(
            "BiasEstimator: band_quality 必须 ∈ (0,1]");
    }
}

bool BiasEstimator::within_at(const pm::Transform4D & a,
                              const pm::Transform4D & b,
                              const std::array<double, 3> & p_ob,
                              double trans_th, double yaw_th)
{
    // ③修法: 机体点残差度量 —— 在 p_ob 处评估两变换的位置差,
    // yaw 解算噪声经力臂的平移伪差在此度量下恒零 (设计文档 v1 4.2)
    const pm::TransformError err = pm::transform_error(
        a, b, std::vector<std::array<double, 3>>{p_ob});
    return err.trans <= trans_th && err.yaw <= yaw_th;
}

pm::Transform4D BiasEstimator::mean_of(const std::vector<pm::Transform4D> & c)
{
    // yaw 以首帧为基准的相对角均值, 防 ±180° 环绕
    // (候选帧相互一致 (< 门限之半), 相对角远离环绕点)
    pm::Transform4D m;
    const double base_yaw = c.front().yaw;
    double sum_yaw_rel = 0.0;
    for (const auto & t : c) {
        m.x += t.x;
        m.y += t.y;
        m.z += t.z;
        sum_yaw_rel += pm::wrap_angle(t.yaw - base_yaw);
    }
    const double n = static_cast<double>(c.size());
    m.x /= n;
    m.y /= n;
    m.z /= n;
    m.yaw = pm::wrap_angle(base_yaw + sum_yaw_rel / n);
    return m;
}

void BiasEstimator::note_observation(SampleTime t_sample, HostTime t_arrival)
{
    last_obs_sample_ = t_sample;
    last_obs_arrival_ = t_arrival;
    // 同源钟假设声明点: 采样戳由本机钟盖 (相机驱动/fc_bridge), 差值即
    // 端到端延迟; 采样戳换设备独立钟时此处需重审 (⑤钟域)
    last_obs_delay_ = delay_assuming_same_clock(t_sample, t_arrival);
    has_observation_ = true;
}

void BiasEstimator::add_observation(const pm::Transform4D & obs,
                                    SampleTime t_sample, HostTime t_arrival,
                                    const std::array<double, 3> & p_ob,
                                    double quality)
{
    // 有限性守卫: NaN/inf 观测一旦进账本, 门控比较对 NaN 恒 false 且
    // 速率 clamp 失效, raw/ctrl 无法恢复 —— 在唯一入口拦截
    if (!(std::isfinite(obs.x) && std::isfinite(obs.y) &&
          std::isfinite(obs.z) && std::isfinite(obs.yaw) &&
          std::isfinite(t_sample.s) && std::isfinite(t_arrival.s) &&
          std::isfinite(p_ob[0]) && std::isfinite(p_ob[1]) &&
          std::isfinite(p_ob[2]) && std::isfinite(quality))) {
        ++invalid_obs_count_;
        return;
    }

    switch (state_) {
        case BiasState::UNINITIALIZED:
            init_candidates_.clear();
            init_candidates_.push_back(obs);
            note_observation(t_sample, t_arrival);
            current_quality_ = clamp01(quality);
            state_ = BiasState::INITIALIZING;
            return;

        case BiasState::INITIALIZING:
            // 初始化一致性确认 (详设 4.7) —— 帧间一致标准与门控候选队列
            // 同口径 (门限之半), 防止用一帧误检完成初始化。
            // 候选可能被 reset 清空 (apply_reset), 此时 obs 成为新起点。
            note_observation(t_sample, t_arrival);    // 初始化候选均视为有效观测
            current_quality_ = clamp01(quality);      // 记账 (初始化期无吸收消费)
            if (!init_candidates_.empty() &&
                within_at(obs, init_candidates_.back(), p_ob,
                          params_.gate_trans_threshold * 0.5,
                          params_.gate_yaw_threshold * 0.5)) {
                init_candidates_.push_back(obs);
                if (static_cast<int>(init_candidates_.size()) >=
                    params_.init_confirm_frames) {
                    // 初始化不走吸收: ctrl/cmd 无先验, raw = ctrl = cmd = 候选均值
                    raw_ = mean_of(init_candidates_);
                    ctrl_ = raw_;
                    cmd_ = raw_;
                    init_candidates_.clear();
                    state_ = BiasState::TRACKING;
                }
            } else {
                init_candidates_.clear();
                init_candidates_.push_back(obs);    // 新帧成为新候选起点
            }
            return;

        case BiasState::TRACKING:
        case BiasState::STALE:
            gate_and_update(obs, t_sample, t_arrival, p_ob, quality);
            return;
    }
}

void BiasEstimator::gate_and_update(const pm::Transform4D & obs,
                                    SampleTime t_sample, HostTime t_arrival,
                                    const std::array<double, 3> & p_ob,
                                    double quality)
{
    // 三带门控 (详设 4.4 + v2 质量链): 度量在本帧机体点 p_ob 处评估
    // (③修法): "账本预测的机体 map 位置 vs 观测的机体 map 位置",
    // yaw 噪声力臂贡献恒零。
    //   err ≤ Δ       正常吸收带: 采纳, q_band = 1
    //   Δ < err ≤ L   降权带: 采纳但降档 (raw 仍进原值 —— 打标不降权,
    //                 账本纯净; 降档消费在 tick 吸收 α·q)
    //   err > L       拒绝带: 候选队列 N 帧确认跳变 (超门野值原样入队
    //                 不钳边界 —— MRS "大异常必须透传" 原则)
    // last_obs_time 只在观测被账本采纳时刷新 —— 被拒绝的观测不算"有效",
    // 误检风暴下账本停更, obs_age 增长直至 STALE 告警 (详设 v2.1, F02)
    const bool in_soft = within_at(obs, raw_, p_ob,
                                   params_.gate_soft_trans_threshold,
                                   params_.gate_soft_yaw_threshold);
    if (in_soft ||
        within_at(obs, raw_, p_ob, params_.gate_trans_threshold,
                  params_.gate_yaw_threshold)) {
        // 被正常路径清空的候选 = 被抛弃的观测, 按帧数计入拒绝 (F13)
        gate_reject_count_ += static_cast<uint32_t>(gate_candidates_.size());
        gate_candidates_.clear();
        raw_ = obs;    // 账本采纳原值 (中值去毛刺 v2 起在 intake 链上, 门控前)
        note_observation(t_sample, t_arrival);
        current_quality_ =
            clamp01(quality) * (in_soft ? 1.0 : params_.band_quality);
        if (state_ == BiasState::STALE) {
            state_ = BiasState::TRACKING;    // 观测被采纳 → 断流恢复
        }
        return;
    }

    // 候选路径 (第 2/3 条)
    if (!gate_candidates_.empty() &&
        within_at(obs, gate_candidates_.back(), p_ob,
                  params_.gate_trans_threshold * 0.5,
                  params_.gate_yaw_threshold * 0.5)) {
        // 与队列内候选一致 → 累积
        gate_candidates_.push_back(obs);
        if (static_cast<int>(gate_candidates_.size()) >=
            params_.gate_confirm_frames) {
            // 确认为真实跳变: raw ← 候选均值, 一次性跳变不渐变。
            // 确认的候选帧最终被采纳, 不计入拒绝 —— 合法重定位事件
            // 不污染误检信号 (F13)
            const pm::Transform4D old_raw = raw_;
            raw_ = mean_of(gate_candidates_);
            gate_candidates_.clear();
            record_jump(old_raw, raw_);
            note_observation(t_sample, t_arrival);
            // 跳变确认是拒绝带的出口 (合法重定位), 不属于降权带 —— 只记源质量
            current_quality_ = clamp01(quality);
            if (state_ == BiasState::STALE) {
                state_ = BiasState::TRACKING;    // 候选确认 → 断流恢复
            }
        }
    } else {
        // 队列空 (首个突变帧), 或与候选不一致 → 被重置的旧候选计入拒绝,
        // 队列重置为仅含新帧
        gate_reject_count_ += static_cast<uint32_t>(gate_candidates_.size());
        gate_candidates_.clear();
        gate_candidates_.push_back(obs);
    }
}

void BiasEstimator::record_jump(const pm::Transform4D & old_raw,
                                const pm::Transform4D & new_raw)
{
    const double dx = new_raw.x - old_raw.x;
    const double dy = new_raw.y - old_raw.y;
    const double dz = new_raw.z - old_raw.z;
    last_jump_trans_ = std::sqrt(dx * dx + dy * dy + dz * dz);
    last_jump_yaw_ = std::fabs(pm::wrap_angle(new_raw.yaw - old_raw.yaw));
    ++jump_count_;
    ++reference_iteration_;    // 跳变确认 = 参考系事件 (SOURCE_JUMP)
}

void BiasEstimator::absorb_toward(const pm::Transform4D & target,
                                  pm::Transform4D & state, double dt,
                                  double tau, double rate_trans, double rate_yaw,
                                  const std::vector<std::array<double, 3>> & eval_points,
                                  double quality)
{
    // 吸收律 (详设 4.5, ctrl/cmd 共用): 一阶低通 + 速率硬上限。
    // v2 软降权 (5.3): α ← α·q, 等价该帧 τ 临时拉长为 τ/q —— 降权在前
    // (线性缩放保幅值信息), 下方速率钳位退化为最后兜底
    const double alpha = dt / tau * quality;
    double step_x = (target.x - state.x) * alpha;
    double step_y = (target.y - state.y) * alpha;
    double step_z = (target.z - state.z) * alpha;
    double step_yaw = pm::wrap_angle(target.yaw - state.yaw) * alpha;

    // yaw 先走独立预算
    const double yaw_limit = rate_yaw * dt;
    step_yaw = std::max(-yaw_limit, std::min(yaw_limit, step_yaw));

    // 平移预算以机体点位移度量 (③修法, 设计文档 v1 4.3-2): 物理语义 =
    // "机体 setpoint 被拉动的速率 ≤ rate_trans", yaw 步进经力臂的拉动
    // 计入同一预算 → 超限时全通道等比缩步 (保方向缩模长)。
    // 空 eval_points 退化为原点评估 = 参数空间旧钳位 (缓冲空回退路径)。
    // 两轮缩放: yaw 弦长对缩放系数是凹函数, 一轮线性缩放后实际位移可略
    // 超预算 (残差 ~ r·ψ³/24, ψ ≤ rate_yaw·max_tick_dt), 第二轮吃掉残差
    const double trans_limit = rate_trans * dt;
    for (int pass = 0; pass < 2; ++pass) {
        pm::Transform4D cand = state;
        cand.x += step_x;
        cand.y += step_y;
        cand.z += step_z;
        cand.yaw = pm::wrap_angle(state.yaw + step_yaw);
        const double moved =
            pm::transform_error(cand, state, eval_points).trans;
        if (moved <= trans_limit || moved <= 0.0) {
            break;
        }
        const double scale = trans_limit / moved;
        step_x *= scale;
        step_y *= scale;
        step_z *= scale;
        step_yaw *= scale;
    }

    state.x += step_x;
    state.y += step_y;
    state.z += step_z;
    state.yaw = pm::wrap_angle(state.yaw + step_yaw);
}

void BiasEstimator::tick(HostTime t_now,
                         const std::vector<std::array<double, 3>> & eval_points)
{
    // dt 防御: 首拍只记基准; 时钟回退刷新基准; 单拍上限 max_tick_dt
    // (定时器挂起恢复时防 ctrl/cmd 单步大跳)
    double dt = 0.0;
    if (has_ticked_ && t_now.s > last_tick_.s) {
        dt = std::min(t_now.s - last_tick_.s, params_.max_tick_dt);
    }
    last_tick_ = t_now;
    has_ticked_ = true;

    // STALE 判定先于步进: 进入 STALE 当拍即冻结。超时以**到达刻**计
    // (⑤钟域纪律): 深延迟链路下健康观测流不被误判断流, 断流判定
    // 与链路延迟解耦
    if (state_ == BiasState::TRACKING && has_observation() &&
        age_of(last_obs_arrival_, t_now) > params_.observation_timeout) {
        state_ = BiasState::STALE;
    }
    if (state_ != BiasState::TRACKING || dt <= 0.0) {
        return;    // STALE 双通道冻结 / 未初始化无状态 / 无时间推进
    }

    // ctrl 慢通道 (感知侧, 稳) 与 cmd 快通道 (指令侧, 贴 raw) 同律不同参;
    // 双通道同吃软降权 q_eff (5.3: "ctrl/cmd 轨降权在前")
    absorb_toward(raw_, ctrl_, dt, params_.absorb_time_constant,
                  params_.max_correction_rate_trans,
                  params_.max_correction_rate_yaw, eval_points,
                  current_quality_);
    absorb_toward(raw_, cmd_, dt, params_.cmd_absorb_time_constant,
                  params_.cmd_max_correction_rate_trans,
                  params_.cmd_max_correction_rate_yaw, eval_points,
                  current_quality_);
}

void BiasEstimator::apply_reset(const pm::Transform4D & d)
{
    // UNINITIALIZED 无内部状态可补偿, 也不记事件 —— 详设 4.6: 该状态
    // "仅更新 counter 缓存" (缓存在节点薄壳完成, F15)
    if (state_ == BiasState::UNINITIALIZED) {
        return;
    }

    ++reset_event_count_;
    ++reference_iteration_;    // reset 补偿 = 参考系事件 (EKF_RESET)
    last_reset_trans_ = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    last_reset_yaw_ = std::fabs(pm::wrap_angle(d.yaw));

    if (state_ == BiasState::INITIALIZING) {
        // 初始化候选是旧坐标系下的观测, 清空重新累积 (详设 4.6:
        // 简单且安全, 初始化耗时仅数帧); 状态保持 INITIALIZING
        init_candidates_.clear();
        return;
    }

    // TRACKING / STALE:
    // raw/ctrl/cmd ← ·D⁻¹: 机体 map 系位姿跨 reset 连续
    //   T_map_base = (raw·D⁻¹)·(D·T_odom_base) = raw·T_odom_base
    // STALE 也执行 —— 账本的坐标系基准必须跟随 reset, 否则断流恢复
    // 时 raw 与首帧观测相差整个 delta (详设 4.6 与状态机的交互)
    const pm::Transform4D d_inv = pm::inverse(d);
    raw_ = pm::compose(raw_, d_inv);
    ctrl_ = pm::compose(ctrl_, d_inv);
    cmd_ = pm::compose(cmd_, d_inv);
    gate_candidates_.clear();    // 旧坐标系下的候选作废
    // (中值窗清空由宿主对 obs_intake 链 reset_all 完成 —— 节点在
    //  apply_reset 路径已接线, 链上历史同属旧坐标系)
}

double BiasEstimator::divergence_trans(
    const std::vector<std::array<double, 3>> & eval_points) const
{
    // ③修法: "感知位置与指令位置在机体处的分歧" —— raw/ctrl 的 yaw 分歧
    // 经力臂产生的机体位置分歧计入; 空集 = 参数空间平移差 (旧口径)
    return pm::transform_error(raw_, ctrl_, eval_points).trans;
}

double BiasEstimator::divergence_yaw() const
{
    return std::fabs(pm::wrap_angle(raw_.yaw - ctrl_.yaw));
}

double BiasEstimator::divergence_cmd_trans(
    const std::vector<std::array<double, 3>> & eval_points) const
{
    return pm::transform_error(cmd_, ctrl_, eval_points).trans;
}

double BiasEstimator::divergence_cmd_yaw() const
{
    return std::fabs(pm::wrap_angle(cmd_.yaw - ctrl_.yaw));
}

}  // namespace map_odom_bias

/**
 * @file obs_intake.hpp
 * @brief 输入侧修正源抽象: 观测窄接口 + 两票 processor 链 (设计文档 v1
 *        五节 5.2, 裁决 C1 = 接口 v1 就位、降权消费 v2), 零 ROS 依赖
 *
 * 两票语义 (tag_transform 守卫雏形的成文化):
 *   - usable (生死票): false = 整帧废弃, 链短路, 不喂估计器
 *   - clean  (降权票): false = 帧可用但质量存疑, processor 自行降 quality
 *     (quality ∈ [0,1] 随帧透传; v1 估计器不消费, v2 软降权 α←α·q 接入)
 *
 * 链序由装配顺序显式给定 (中值前置于钳位类的顺序约束成文)。
 * processor 可读宿主估计器状态 (钳位类判异常的必要引用)。
 * v1 首发链 = FiniteGuard; v2 质量链追加 MedianWindow (中值去毛刺,
 * 自估计器 raw_median_window 迁入) 与 TagCountQuality (detector 先验
 * → quality 分档, 软降权消费的上游信号源)。
 */

#ifndef MAP_ODOM_BIAS__CORE__OBS_INTAKE_HPP_
#define MAP_ODOM_BIAS__CORE__OBS_INTAKE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "map_odom_bias/core/bias_estimator.hpp"
#include "map_odom_bias/core/pose_math.hpp"
#include "map_odom_bias/core/time_types.hpp"

namespace map_odom_bias
{

/// 观测窄接口 (设计文档 v1 3.2): 源侧适配层负责把任意消息形态换算成
/// 本结构, 核心库不做多消息类型 switch
struct GlobalPoseObservation
{
    SampleTime t_sample;                       // 采样刻 (进账本/门控)
    HostTime t_arrival;                        // 到达刻 (判存活)
    pose_math::Transform4D T_obs;              // 观测到的 T_map_odom (4DoF)
    std::array<double, 3> p_ob{{0.0, 0.0, 0.0}};    // 机体 odom 位置 (③评估点)
    double quality{1.0};                       // [0,1], 缺省满质量
    int tag_count{-1};                         // 本帧解算 tag 数 (detector
                                               // 先验); -1 = 上游未提供
};

/// 两票: 降权票 (clean) / 生死票 (usable)
struct ProcResult
{
    bool clean{true};
    bool usable{true};
};

class ObsProcessor
{
public:
    virtual ~ObsProcessor() = default;
    /// 可原地修改 obs (降 quality / 滤波); host 供钳位类判异常参考
    virtual ProcResult process(GlobalPoseObservation & obs,
                               const BiasEstimator & host) = 0;
    /// 内部状态清空 (reset 事件后旧坐标系历史作废; 按值拷贝导致
    /// reset 失效是 MRS 已知坑, 实现必须真正清到持有状态的实例)
    virtual void reset() = 0;
};

/// 链执行器: 按装配顺序执行, usable=false 短路废弃
class ObsIntake
{
public:
    void add(std::unique_ptr<ObsProcessor> p)
    {
        chain_.push_back(std::move(p));
    }

    /// @return 帧是否可用 (false = 已废弃并计数)
    bool run(GlobalPoseObservation & obs, const BiasEstimator & host)
    {
        for (auto & p : chain_) {
            const ProcResult r = p->process(obs, host);
            if (!r.usable) {
                ++dropped_count_;
                return false;
            }
        }
        return true;
    }

    void reset_all()
    {
        for (auto & p : chain_) {
            p->reset();
        }
    }

    uint32_t dropped_count() const { return dropped_count_; }
    std::size_t size() const { return chain_.size(); }

private:
    std::vector<std::unique_ptr<ObsProcessor>> chain_;
    uint32_t dropped_count_{0};
};

/// 有限性守卫: 任意字段非有限 → 生死票废弃 (估计器入口守卫保留作纵深,
/// 本 processor 是链上第一道且带独立计数)
class FiniteGuard : public ObsProcessor
{
public:
    ProcResult process(GlobalPoseObservation & obs,
                       const BiasEstimator & /*host*/) override
    {
        const bool finite =
            std::isfinite(obs.T_obs.x) && std::isfinite(obs.T_obs.y) &&
            std::isfinite(obs.T_obs.z) && std::isfinite(obs.T_obs.yaw) &&
            std::isfinite(obs.p_ob[0]) && std::isfinite(obs.p_ob[1]) &&
            std::isfinite(obs.p_ob[2]) &&
            std::isfinite(obs.t_sample.s) && std::isfinite(obs.t_arrival.s) &&
            std::isfinite(obs.quality);
        ProcResult r;
        r.usable = finite;
        r.clean = finite;
        return r;
    }

    void reset() override {}    // 无内部状态
};

/// 观测中值去毛刺窗口 (v2, 自估计器 raw_median_window 迁入 —— 设计文档
/// v1 5.2, MRS median_filter 同位): 每通道独立中值, yaw 以窗口首元素为
/// 基准防 ±180° 环绕。⚠位置语义与迁移前不同: 原实现在门控后账本内,
/// 现在门控前观测流上 —— 毛刺在进门控前滤除, 真跳变经中值延迟
/// ~window/2 帧后透传候选队列 (window<=1 直通, 默认关闭时零差异)。
/// 窗内历史属当前参考系: reset 事件后必须经 reset_all 真清到实例
/// (MRS 按值拷贝 reset 失效坑的对治; 节点以 iteration 变化驱动)
class MedianWindow : public ObsProcessor
{
public:
    explicit MedianWindow(int window)
    : window_(std::max(1, window))
    {
    }

    ProcResult process(GlobalPoseObservation & obs,
                       const BiasEstimator & /*host*/) override
    {
        if (window_ <= 1) {
            return ProcResult{};    // 直通
        }
        buf_.push_back(obs.T_obs);
        while (static_cast<int>(buf_.size()) > window_) {
            buf_.pop_front();
        }
        obs.T_obs = median_of(buf_);
        return ProcResult{};
    }

    void reset() override { buf_.clear(); }

private:
    static pose_math::Transform4D median_of(
        const std::deque<pose_math::Transform4D> & w)
    {
        // 每通道独立中值 (偶数窗口取中间两值平均); yaw 以首元素为基准的
        // 相对角中值 (窗口内观测经上游门控约束, 相对角远离环绕点)
        auto channel_median = [](std::vector<double> & v) {
            std::sort(v.begin(), v.end());
            const std::size_t n = v.size();
            return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
        };
        std::vector<double> xs, ys, zs, yaws;
        const double base_yaw = w.front().yaw;
        for (const auto & t : w) {
            xs.push_back(t.x);
            ys.push_back(t.y);
            zs.push_back(t.z);
            yaws.push_back(pose_math::wrap_angle(t.yaw - base_yaw));
        }
        pose_math::Transform4D m;
        m.x = channel_median(xs);
        m.y = channel_median(ys);
        m.z = channel_median(zs);
        m.yaw = pose_math::wrap_angle(base_yaw + channel_median(yaws));
        return m;
    }

    int window_;
    std::deque<pose_math::Transform4D> buf_;
};

/// detector 先验 → quality 分档 (v2, 设计文档 v1 5.2): 本帧解算 tag 数是
/// 现成的可信度信号 (实测运动帧 1~3 tag, 单 tag = PnP 双解翻转风险档)。
/// tag_count<0 (上游未提供) 直通满质量; ≥3 满 / ==2 双 tag 档 / ≤1 单
/// tag 档 (0-tag 有 pose 属上游异常, 并入最低档保守处理, 生死票留给
/// FiniteGuard —— 无失活路径原则)。降档帧 clean=false (MRS ok_flag
/// 语义对齐), quality 乘法复合随帧透传。无内部状态。
class TagCountQuality : public ObsProcessor
{
public:
    /**
     * @throw std::invalid_argument 分档因子非法 (须 0 < single ≤ dual ≤ 1)
     *        —— 加载期硬断言, 配置错误 fail-fast 不带病运行
     */
    TagCountQuality(double single_tag_quality, double dual_tag_quality)
    : single_(single_tag_quality), dual_(dual_tag_quality)
    {
        if (!(single_ > 0.0 && single_ <= dual_ && dual_ <= 1.0)) {
            throw std::invalid_argument(
                "TagCountQuality: 须 0 < single ≤ dual ≤ 1");
        }
    }

    ProcResult process(GlobalPoseObservation & obs,
                       const BiasEstimator & /*host*/) override
    {
        double q = 1.0;
        if (obs.tag_count >= 0) {
            if (obs.tag_count >= 3) {
                q = 1.0;
            } else if (obs.tag_count == 2) {
                q = dual_;
            } else {
                q = single_;
            }
        }
        obs.quality *= q;
        ProcResult r;
        r.clean = (q >= 1.0);
        return r;
    }

    void reset() override {}    // 无内部状态

private:
    double single_;
    double dual_;
};

}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__CORE__OBS_INTAKE_HPP_

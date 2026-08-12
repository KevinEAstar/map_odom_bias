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
 * v1 首发链 = FiniteGuard (实际在用, 无失活路径原则 —— TagQuality/
 * MedianWindow 等 v2 随降权消费一起实装, 不预埋关闭的死代码)。
 */

#ifndef MAP_ODOM_BIAS__CORE__OBS_INTAKE_HPP_
#define MAP_ODOM_BIAS__CORE__OBS_INTAKE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
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

}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__CORE__OBS_INTAKE_HPP_

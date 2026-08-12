/**
 * @file odom_buffer.hpp
 * @brief odom 历史缓冲 + 时间戳对齐插值 (详设 4.2 节), 纯逻辑无 ROS 依赖
 *
 * 职责:
 *   - 缓存最近 duration 秒内的 odom 位姿样本 (时间升序)
 *   - 按图像时间戳查询: 相邻样本间位置线性插值 + 姿态球面插值 (slerp)
 *   - 边界: 早于最旧样本 → TOO_OLD; 晚于最新样本 ≤ max_extrapolation →
 *     零阶保持 (取最新样本); 超出 → TOO_NEW
 *   - EKF reset 时 clear_and_settle(): 清空 + 静默窗口 (详设 4.6 节附带动作
 *     2/3 —— 缓冲内样本全部表达在旧坐标系, 跨 reset 插值会产生错误位姿;
 *     两条订阅链路独立 DDS 无到达顺序保证, 窗口内到达的旧系样本直接丢弃)
 *
 * 时间统一为 double 秒 (本地 ROS 钟, 由调用方转换); 观测的最大可用年龄由
 * duration 隐式决定, 不设独立门限 (详设 v2: max_observation_age 已删除)。
 * 健康计数 (too_old/too_new/乱序/静默丢弃) 内聚在本类, 供 ~/status 读取。
 */

#ifndef MAP_ODOM_BIAS__CORE__ODOM_BUFFER_HPP_
#define MAP_ODOM_BIAS__CORE__ODOM_BUFFER_HPP_

#include <cstdint>
#include <deque>

#include "map_odom_bias/core/pose_math.hpp"

namespace map_odom_bias
{

/// odom 位姿样本 (ENU 世界系位置 + FLU 机体姿态)
struct OdomSample
{
    double t{0.0};    // header.stamp, 本地 ROS 钟, 秒
    pose_math::Pose pose;
};

class OdomBuffer
{
public:
    enum class QueryResult
    {
        OK,
        TOO_OLD,     // t 早于缓冲最旧样本
        TOO_NEW,     // t 晚于最新样本超过 max_extrapolation
        EMPTY        // 缓冲为空 (含 reset 清空后), 健康计数并入 too_old
    };

    /**
     * @param duration          缓冲时长 s (odom_buffer_duration)
     * @param max_extrapolation 图像戳晚于最新 odom 时允许的外推上限 s
     */
    OdomBuffer(double duration, double max_extrapolation);

    /**
     * @brief 追加样本 (要求时间严格递增)
     * @return false = 被丢弃 (时间戳乱序/回退, 或处于 reset 静默窗口内)
     */
    bool push(const OdomSample & sample);

    /**
     * @brief 按时间戳查询位姿 (线性插值 + slerp)
     * @note 非 const: TOO_OLD/TOO_NEW/EMPTY 时内部健康计数自增
     */
    QueryResult query(double t, pose_math::Pose * out);

    /**
     * @brief EKF reset: 清空缓冲并进入静默窗口 (详设 4.6)
     * @param settle_until 窗口截止时刻; 在此之前的样本 push 时直接丢弃。
     *        判定用样本戳 (= fc_bridge 到达时刻本地钟, 与本节点到达时刻
     *        进程间差 µs 级, 等效且纯逻辑可测)
     */
    void clear_and_settle(double settle_until);

    void clear();

    std::size_t size() const { return buf_.size(); }
    bool empty() const { return buf_.empty(); }
    double oldest_time() const { return buf_.empty() ? 0.0 : buf_.front().t; }
    double newest_time() const { return buf_.empty() ? 0.0 : buf_.back().t; }
    /// 最新样本位姿 (③修法钳位/divergence 的机体评估点来源;
    /// 调用方须先以 empty() 确认非空)
    const pose_math::Pose & newest_pose() const { return buf_.back().pose; }

    // 健康计数 (详设 4.8: obs_too_old / obs_too_new 的数据源)
    uint32_t too_old_count() const { return too_old_count_; }
    uint32_t too_new_count() const { return too_new_count_; }
    uint32_t disorder_drop_count() const { return disorder_drop_count_; }
    uint32_t settle_drop_count() const { return settle_drop_count_; }
    uint32_t invalid_drop_count() const { return invalid_drop_count_; }

private:
    std::deque<OdomSample> buf_;    // 时间严格升序
    double duration_;
    double max_extrapolation_;
    double settle_until_{0.0};

    uint32_t too_old_count_{0};
    uint32_t too_new_count_{0};
    uint32_t disorder_drop_count_{0};
    uint32_t settle_drop_count_{0};
    uint32_t invalid_drop_count_{0};
};

}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__CORE__ODOM_BUFFER_HPP_

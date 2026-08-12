/**
 * @file time_types.hpp
 * @brief 钟域 strong typedef (设计文档 v1 七节, ⑤钟域声明), 零 ROS 依赖
 *
 * 三类钟的类型级约束:
 *   - SampleTime: 传感采样刻 (FC 报文换算 / 图像曝光戳 / header.stamp)
 *       —— 进估计: 门控 / 账本 / 缓冲配对全用采样戳
 *   - HostTime:   本机到达/处理刻 (节点钟 now())
 *       —— 判存活: STALE 超时 / tick 步进 / 静默窗全用到达刻
 *   - 相机独立单调钟在源侧适配层换算为 SampleTime, 不进核心库
 *
 * 跨域运算只经显式函数, 裸减法编译不过 —— 现状 "图像戳与节点钟同域"
 * 是未声明假设 (相机链盖戳方式一变即错拍), 类型化后该假设必须在
 * *_assuming_same_clock 调用点显式声明。
 */

#ifndef MAP_ODOM_BIAS__CORE__TIME_TYPES_HPP_
#define MAP_ODOM_BIAS__CORE__TIME_TYPES_HPP_

namespace map_odom_bias
{

/// 传感采样刻 (进估计: 门控/账本/缓冲配对)
struct SampleTime
{
    double s{0.0};
};

/// 本机到达/处理刻 (判存活: STALE/tick/静默窗)
struct HostTime
{
    double s{0.0};
};

/// 同域年龄: 到达刻之间的流逝 (STALE 超时判定口径)
inline double age_of(HostTime earlier, HostTime now)
{
    return now.s - earlier.s;
}

/// 同域年龄: 采样刻之间的流逝 (缓冲裁剪/配对窗口口径)
inline double age_of(SampleTime earlier, SampleTime now)
{
    return now.s - earlier.s;
}

/**
 * @brief 跨域差值 —— 显式声明"同源钟假设"的唯一出口
 *
 * 采样戳由本机钟盖 (相机驱动 now() / fc_bridge 到达刻) 时两钟同域,
 * 差值 = 端到端延迟 (obs_delay 可观测项)。假设由调用方在调用点声明;
 * 采样戳换成设备独立钟 (相机单调钟直通) 时, 该函数的调用点即全部
 * 需要重审的位置 —— 这正是类型化的目的。
 */
inline double delay_assuming_same_clock(SampleTime sample, HostTime arrival)
{
    return arrival.s - sample.s;
}

/// 跨域转换 (同上假设): 到达刻当采样刻用 (reset 静默窗对样本戳的判定)
inline SampleTime as_sample_time_assuming_same_clock(HostTime t)
{
    return SampleTime{t.s};
}

}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__CORE__TIME_TYPES_HPP_

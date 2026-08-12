/**
 * @file pose_math.hpp
 * @brief 纯数学工具: 四元数最小运算 + 4 自由度变换 (map→odom 偏置层专用)
 *
 * 本文件与 ROS 零依赖, 是 OdomBuffer / BiasEstimator 可脱离 ROS 单元测试的
 * 结构前提 (详设第五节)。
 *
 * 约定:
 *   - 四元数 Hamilton 约定, w 在前; 旋转类函数内部做防御性归一化
 *   - Transform4D 表示 T = [Rz(yaw), t] 的 4 自由度刚体变换 (x/y/z/yaw),
 *     roll/pitch 恒零 —— EKF2 的 roll/pitch 由 IMU 重力锚定不漂移,
 *     漂移只存在于 x/y/z/yaw (架构文档 3.2)
 *   - 全部 yaw 运算按最短角路径, 差值归一化至 (-pi, pi]
 */

#ifndef MAP_ODOM_BIAS__CORE__POSE_MATH_HPP_
#define MAP_ODOM_BIAS__CORE__POSE_MATH_HPP_

#include <array>

namespace map_odom_bias
{
namespace pose_math
{

static constexpr double kPi = 3.14159265358979323846;

/// 角度归一化至 (-pi, pi]
double wrap_angle(double a);

/// 单位四元数, Hamilton 约定 (w 在前)
struct Quat
{
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Quat quat_normalize(const Quat & q);
Quat quat_mul(const Quat & a, const Quat & b);
/// 单位四元数的逆 (共轭)
Quat quat_conjugate(const Quat & q);
/// 球面插值, u ∈ [0,1]; 自动取最短路径 (dot<0 翻转)
Quat quat_slerp(const Quat & a, const Quat & b, double u);
/// 绕竖轴 (Z-up) 旋转 yaw 的四元数
Quat quat_from_yaw(double yaw);
/// 提取绕竖轴分量 (ZYX 欧拉的 Z): atan2(2(wz+xy), 1-2(y²+z²))
double yaw_from_quat(const Quat & q);
/// v' = R(q) · v
std::array<double, 3> quat_rotate(const Quat & q, const std::array<double, 3> & v);

/// SE(3) 位姿: 世界系位置 + 机体姿态
struct Pose
{
    std::array<double, 3> p{{0.0, 0.0, 0.0}};
    Quat q;
};

/// 4 自由度刚体变换 T = [Rz(yaw), (x,y,z)]
struct Transform4D
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
};

/// c = a · b (先施加 b 再施加 a): c.p = a.p + Rz(a.yaw)·b.p, c.yaw = a.yaw + b.yaw
Transform4D compose(const Transform4D & a, const Transform4D & b);
/// a⁻¹ = [Rz(-yaw), -Rz(-yaw)·p]
Transform4D inverse(const Transform4D & a);
/// p' = Rz(T.yaw)·p + T.p
std::array<double, 3> apply(const Transform4D & t, const std::array<double, 3> & p);
/// 位姿整体变换: {Rz·p + t, q_yaw ⊗ q}
Pose apply_to_pose(const Transform4D & t, const Pose & pose);

/**
 * @brief 4 自由度偏差观测 (详设 4.3, v2.1 口径)
 *
 *   两位姿各自投影到 4DoF (yaw = 绕竖轴分量, roll/pitch 丢弃) 后组合:
 *   obs = T4(map_base) · T4(odom_base)⁻¹
 *       = [p_mb − Rz(yaw_mb − yaw_ob)·p_ob, yaw_mb − yaw_ob]
 *   平移与输出 yaw 用同一纯 yaw 旋转反解, roll/pitch 噪声对旋转与
 *   平移通道同时隔离 (完整 R_obs 反解会把 roll/pitch 经力臂注入平移)。
 *
 * @param map_base  观测: 机体在 map 系的位姿 (定位节点输出)
 * @param odom_base 同一时刻机体在 odom 系的位姿 (OdomBuffer 插值)
 */
Transform4D bias_observation(const Pose & map_base, const Pose & odom_base);

/**
 * @brief EKF 航向 reset 的 odom 系改写增量 D (详设 4.6)
 *
 *   D = [Rz(Δψ), (I − Rz(Δψ))·p_ob] —— odom 系绕机体当前位置旋转,
 *   机体位置坐标不动 (apply(D, p_ob) == p_ob)。
 *   位置 reset 的 D = [I, Δp] 直接以 Transform4D{Δp, 0} 表达, 不设专函数。
 *
 * @param dpsi_enu ENU yaw 增量 (NED 航向 delta 取反后传入)
 * @param p_ob_enu 机体在 odom 系的当前位置 (reset 报文位置转 ENU)
 */
Transform4D make_heading_reset_delta(double dpsi_enu,
                                     const std::array<double, 3> & p_ob_enu);

}  // namespace pose_math
}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__CORE__POSE_MATH_HPP_

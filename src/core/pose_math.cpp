/**
 * @file pose_math.cpp
 * @brief pose_math.hpp 实现 (纯数学, 无 ROS 依赖)
 */

#include "map_odom_bias/core/pose_math.hpp"

#include <cmath>

namespace map_odom_bias
{
namespace pose_math
{

double wrap_angle(double a)
{
    a = std::fmod(a + kPi, 2.0 * kPi);    // (-2pi, 2pi)
    if (a <= 0.0) {
        a += 2.0 * kPi;                    // (0, 2pi]
    }
    return a - kPi;                        // (-pi, pi]
}

Quat quat_normalize(const Quat & q)
{
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12) {
        return Quat{};    // 退化输入返回单位四元数
    }
    return Quat{q.w / n, q.x / n, q.y / n, q.z / n};
}

Quat quat_mul(const Quat & a, const Quat & b)
{
    return Quat{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Quat quat_conjugate(const Quat & q)
{
    return Quat{q.w, -q.x, -q.y, -q.z};
}

Quat quat_slerp(const Quat & a_in, const Quat & b_in, double u)
{
    Quat a = quat_normalize(a_in);
    Quat b = quat_normalize(b_in);
    double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    // 双倍覆盖: dot<0 时翻转 b 取最短路径
    if (dot < 0.0) {
        b = Quat{-b.w, -b.x, -b.y, -b.z};
        dot = -dot;
    }
    if (dot > 0.9995) {
        // 夹角极小, 线性插值 + 归一化 (避免 sin 除零)
        return quat_normalize(Quat{
            a.w + u * (b.w - a.w),
            a.x + u * (b.x - a.x),
            a.y + u * (b.y - a.y),
            a.z + u * (b.z - a.z)});
    }
    const double theta = std::acos(dot);
    const double sin_theta = std::sin(theta);
    const double wa = std::sin((1.0 - u) * theta) / sin_theta;
    const double wb = std::sin(u * theta) / sin_theta;
    return quat_normalize(Quat{
        wa * a.w + wb * b.w,
        wa * a.x + wb * b.x,
        wa * a.y + wb * b.y,
        wa * a.z + wb * b.z});
}

Quat quat_from_yaw(double yaw)
{
    return Quat{std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
}

double yaw_from_quat(const Quat & q_in)
{
    const Quat q = quat_normalize(q_in);
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::array<double, 3> quat_rotate(const Quat & q_in, const std::array<double, 3> & v)
{
    // v' = v + w·t + q_vec × t, 其中 t = 2·(q_vec × v)
    const Quat q = quat_normalize(q_in);
    const double tx = 2.0 * (q.y * v[2] - q.z * v[1]);
    const double ty = 2.0 * (q.z * v[0] - q.x * v[2]);
    const double tz = 2.0 * (q.x * v[1] - q.y * v[0]);
    return {{v[0] + q.w * tx + (q.y * tz - q.z * ty),
             v[1] + q.w * ty + (q.z * tx - q.x * tz),
             v[2] + q.w * tz + (q.x * ty - q.y * tx)}};
}

Transform4D compose(const Transform4D & a, const Transform4D & b)
{
    const double c = std::cos(a.yaw);
    const double s = std::sin(a.yaw);
    Transform4D out;
    out.x = a.x + c * b.x - s * b.y;
    out.y = a.y + s * b.x + c * b.y;
    out.z = a.z + b.z;
    out.yaw = wrap_angle(a.yaw + b.yaw);
    return out;
}

Transform4D inverse(const Transform4D & a)
{
    const double c = std::cos(a.yaw);
    const double s = std::sin(a.yaw);
    Transform4D out;
    // -Rz(-yaw)·p: Rz(-yaw) = [c s; -s c]
    out.x = -(c * a.x + s * a.y);
    out.y = -(-s * a.x + c * a.y);
    out.z = -a.z;
    out.yaw = wrap_angle(-a.yaw);
    return out;
}

std::array<double, 3> apply(const Transform4D & t, const std::array<double, 3> & p)
{
    const double c = std::cos(t.yaw);
    const double s = std::sin(t.yaw);
    return {{t.x + c * p[0] - s * p[1],
             t.y + s * p[0] + c * p[1],
             t.z + p[2]}};
}

Pose apply_to_pose(const Transform4D & t, const Pose & pose)
{
    Pose out;
    out.p = apply(t, pose.p);
    out.q = quat_normalize(quat_mul(quat_from_yaw(t.yaw), pose.q));
    return out;
}

Transform4D bias_observation(const Pose & map_base, const Pose & odom_base)
{
    // 两位姿各自投影到 4DoF 后在 4DoF 域组合: obs = T4(map_base) · T4(odom_base)⁻¹,
    // 等价于 p_obs = p_mb − Rz(yaw_obs)·p_ob, yaw_obs = yaw_mb − yaw_ob。
    // 平移必须用与输出 yaw 一致的纯 yaw 旋转反解 —— 若先算完整 SE(3) 的
    // T_mb·T_ob⁻¹ 再取平移原值, 被丢弃的 roll/pitch 噪声会经力臂 |p_ob|
    // 重新注入 x/y/z (z 误差 ≈ |p_ob 水平|·sin(pitch 噪声), 6m 力臂 + 2°
    // 即 0.2m 级; 对抗复核 F01, probe 验证 yaw-first 使该泄漏精确归零)
    Transform4D t_mb;
    t_mb.x = map_base.p[0];
    t_mb.y = map_base.p[1];
    t_mb.z = map_base.p[2];
    t_mb.yaw = yaw_from_quat(map_base.q);
    Transform4D t_ob;
    t_ob.x = odom_base.p[0];
    t_ob.y = odom_base.p[1];
    t_ob.z = odom_base.p[2];
    t_ob.yaw = yaw_from_quat(odom_base.q);
    return compose(t_mb, inverse(t_ob));
}

Transform4D make_heading_reset_delta(double dpsi_enu,
                                     const std::array<double, 3> & p_ob_enu)
{
    // PX4 航向 reset 保持位置坐标不动 → odom 系等效绕机体当前位置旋转:
    //   D = [Rz(Δψ), (I − Rz(Δψ))·p_ob], 满足 apply(D, p_ob) == p_ob
    const double c = std::cos(dpsi_enu);
    const double s = std::sin(dpsi_enu);
    Transform4D d;
    d.x = p_ob_enu[0] - (c * p_ob_enu[0] - s * p_ob_enu[1]);
    d.y = p_ob_enu[1] - (s * p_ob_enu[0] + c * p_ob_enu[1]);
    d.z = 0.0;    // 绕竖轴旋转不产生 z 平移
    d.yaw = wrap_angle(dpsi_enu);
    return d;
}

}  // namespace pose_math
}  // namespace map_odom_bias

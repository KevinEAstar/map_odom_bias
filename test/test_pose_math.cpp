/**
 * test_pose_math.cpp
 * pose_math 机体点残差度量 (transform_error) 性质测试。
 * 覆盖设计文档 v1 四节 (③度量空间修法):
 *   - 恒等式 apply(T_obs, p_ob) ≡ p_mb (bias_observation 构造精确成立)
 *   - 力臂免疫: yaw 解算噪声 δψ 下, 变换参数空间平移差随力臂 |p_ob|
 *     线性增长 (2·sin(δψ/2)·r), 机体点残差恒零
 *   - 真平移误差如实计量, 与力臂无关
 *   - transform_error 语义: 多评估点取 max / 空集退化为原点评估
 *     (= 参数空间平移差) / yaw 通道与评估点无关
 * 含 2026-08-06 TF 跳变风暴定量根因的回归用例 (3.15° × 5.03m ≈ 0.277m)。
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "map_odom_bias/core/pose_math.hpp"

namespace pm = map_odom_bias::pose_math;

namespace
{

pm::Transform4D t4(double x, double y, double z, double yaw)
{
    pm::Transform4D t;
    t.x = x;
    t.y = y;
    t.z = z;
    t.yaw = yaw;
    return t;
}

/// ZYX 欧拉 → 四元数 (测试专用, 用于构造带 roll/pitch 噪声的位姿)
pm::Quat quat_from_rpy(double roll, double pitch, double yaw)
{
    const pm::Quat qx{std::cos(roll / 2.0), std::sin(roll / 2.0), 0.0, 0.0};
    const pm::Quat qy{std::cos(pitch / 2.0), 0.0, std::sin(pitch / 2.0), 0.0};
    const pm::Quat qz = pm::quat_from_yaw(yaw);
    return pm::quat_mul(qz, pm::quat_mul(qy, qx));
}

pm::Pose make_pose(double x, double y, double z,
                   double roll, double pitch, double yaw)
{
    pm::Pose p;
    p.p = {{x, y, z}};
    p.q = quat_from_rpy(roll, pitch, yaw);
    return p;
}

double norm3(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// 给 map 侧位姿注入 yaw 解算噪声: 世界 z 轴左乘 δψ, 位置不动
/// (模拟 PnP 解算 yaw 抖动 —— 机体位置观测本身不含该误差)
pm::Pose inject_yaw_noise(const pm::Pose & pose, double dpsi)
{
    pm::Pose out = pose;
    out.q = pm::quat_normalize(pm::quat_mul(pm::quat_from_yaw(dpsi), pose.q));
    return out;
}

}  // namespace

// ---- 恒等式: apply(T_obs, p_ob) ≡ p_mb ------------------------------------

TEST(BiasObservationIdentity, ExactAtConstructionPoint)
{
    // 含 roll/pitch 噪声的一般位姿对: 恒等式仍精确成立 (4DoF 投影
    // 只丢 roll/pitch, 不破坏构造点上的位置一致性)
    const struct
    {
        pm::Pose mb;
        pm::Pose ob;
    } cases[] = {
        {make_pose(1.2, -0.7, 0.5, 0.0, 0.0, 0.9),
         make_pose(0.8, -1.0, 0.4, 0.0, 0.0, 0.6)},
        {make_pose(-3.4, 2.2, 1.1, 0.03, -0.02, -2.8),
         make_pose(-3.0, 2.5, 1.0, 0.01, 0.04, 2.9)},
        {make_pose(5.0, 5.0, -0.2, -0.05, 0.05, 3.1),
         make_pose(0.0, 0.0, 0.0, 0.02, -0.03, -3.1)},
    };
    for (const auto & c : cases) {
        const pm::Transform4D obs = pm::bias_observation(c.mb, c.ob);
        const std::array<double, 3> predicted = pm::apply(obs, c.ob.p);
        EXPECT_NEAR(predicted[0], c.mb.p[0], 1e-9);
        EXPECT_NEAR(predicted[1], c.mb.p[1], 1e-9);
        EXPECT_NEAR(predicted[2], c.mb.p[2], 1e-9);
    }
}

// ---- 力臂免疫: 新度量恒零, 旧度量随力臂线性涨 ------------------------------

TEST(LeverArmImmunity, YawNoiseAmplifiedInParamSpaceButNotAtBodyPoint)
{
    const pm::Transform4D t_true = t4(0.4, -0.25, 0.1, 0.06);
    const double dpsi = 0.055;    // ≈3.15°, 08-06 实测 yaw 噪声 p95
    const double lever_arms[] = {1.0, 5.0, 10.0};

    for (const double r : lever_arms) {
        // 机体在 odom 系离原点 r (水平), 带任意 yaw 与小 roll/pitch
        const pm::Pose ob = make_pose(r * 0.6, r * 0.8, 0.3, 0.01, -0.02, 1.1);
        const pm::Pose mb_true = pm::apply_to_pose(t_true, ob);
        const pm::Pose mb_noisy = inject_yaw_noise(mb_true, dpsi);
        const pm::Transform4D obs = pm::bias_observation(mb_noisy, ob);

        // 旧度量 (变换参数空间平移差): 精确等于 2·sin(δψ/2)·r_horiz
        const double dx = obs.x - t_true.x;
        const double dy = obs.y - t_true.y;
        const double dz = obs.z - t_true.z;
        const double old_metric = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double r_horiz = std::hypot(ob.p[0], ob.p[1]);
        EXPECT_NEAR(old_metric, 2.0 * std::sin(dpsi / 2.0) * r_horiz, 1e-9);

        // 新度量 (机体点残差): yaw 噪声力臂贡献恒零, yaw 通道如实计量 δψ
        const pm::TransformError err =
            pm::transform_error(obs, t_true, {ob.p});
        EXPECT_NEAR(err.trans, 0.0, 1e-9);
        EXPECT_NEAR(err.yaw, dpsi, 1e-9);
    }
}

TEST(LeverArmImmunity, Storm0806Regression)
{
    // 08-06 风暴定量根因回归: yaw 噪声 p95 3.15° × 力臂 5.03m ≈ 0.277m,
    // 旧度量逼近 gate_trans_threshold=0.3 击穿门控; 新度量恒零
    const pm::Transform4D t_true = t4(0.1, 0.2, 0.0, -0.3);
    const double dpsi = 3.15 * pm::kPi / 180.0;
    const double r = 5.03;
    const pm::Pose ob = make_pose(r, 0.0, 0.5, 0.0, 0.0, 0.4);
    const pm::Pose mb_noisy =
        inject_yaw_noise(pm::apply_to_pose(t_true, ob), dpsi);
    const pm::Transform4D obs = pm::bias_observation(mb_noisy, ob);

    const double dx = obs.x - t_true.x;
    const double dy = obs.y - t_true.y;
    const double old_metric = std::hypot(dx, dy);
    EXPECT_GT(old_metric, 0.27);    // 旧度量: 平移伪差逼近 0.3m 门限

    const pm::TransformError err = pm::transform_error(obs, t_true, {ob.p});
    EXPECT_NEAR(err.trans, 0.0, 1e-9);    // 新度量: 力臂免疫
}

TEST(LeverArmImmunity, TrueTranslationErrorMeasuredExactly)
{
    // 真平移观测误差 e 在任意力臂下如实计量: err.trans = ‖e‖
    const pm::Transform4D t_true = t4(0.3, -0.1, 0.05, 0.2);
    const std::array<double, 3> e = {{0.08, -0.06, 0.03}};
    const double e_norm = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);

    for (const double r : {1.0, 5.0, 10.0}) {
        const pm::Pose ob = make_pose(-r * 0.8, r * 0.6, 0.2, 0.0, 0.0, -0.7);
        pm::Pose mb = pm::apply_to_pose(t_true, ob);
        mb.p[0] += e[0];
        mb.p[1] += e[1];
        mb.p[2] += e[2];
        const pm::Transform4D obs = pm::bias_observation(mb, ob);
        const pm::TransformError err = pm::transform_error(obs, t_true, {ob.p});
        EXPECT_NEAR(err.trans, e_norm, 1e-9);
    }
}

// ---- transform_error 函数语义 ----------------------------------------------

TEST(TransformError, MaxOverEvaluationPoints)
{
    // 纯 yaw 差的两变换: 各点差 = 2·sin(Δyaw/2)·r_horiz, 取 max
    const pm::Transform4D a = t4(0.0, 0.0, 0.0, 0.1);
    const pm::Transform4D b = t4(0.0, 0.0, 0.0, 0.0);
    const std::vector<std::array<double, 3>> points = {
        {{1.0, 0.0, 0.0}},
        {{4.0, 3.0, 1.0}},    // r_horiz = 5, 差最大
    };
    const pm::TransformError err = pm::transform_error(a, b, points);
    double expect_max = 0.0;
    for (const auto & p : points) {
        expect_max = std::max(
            expect_max, norm3(pm::apply(a, p), pm::apply(b, p)));
    }
    EXPECT_NEAR(err.trans, expect_max, 1e-12);
    EXPECT_NEAR(err.trans, 2.0 * std::sin(0.05) * 5.0, 1e-9);
}

TEST(TransformError, EmptyPointsFallBackToOrigin)
{
    // 空评估点集 = 在原点评估 = 参数空间平移差 (消费侧缓冲空时的
    // 退化路径, 设计文档 4.3-2)
    const pm::Transform4D a = t4(0.3, -0.4, 0.12, 0.5);
    const pm::Transform4D b = t4(0.1, 0.0, 0.02, -0.2);
    const pm::TransformError err =
        pm::transform_error(a, b, std::vector<std::array<double, 3>>{});
    EXPECT_NEAR(err.trans,
                std::sqrt(0.2 * 0.2 + 0.4 * 0.4 + 0.1 * 0.1), 1e-12);
}

TEST(TransformError, YawChannelIndependentOfPoints)
{
    // yaw 残差不吃力臂, 与评估点无关; 最短角处理 ±π 环绕
    const pm::Transform4D a = t4(1.0, 2.0, 0.0, 3.0);
    const pm::Transform4D b = t4(-1.0, 0.5, 0.3, -3.0);
    const std::vector<std::array<double, 3>> points = {{{7.0, -2.0, 1.0}}};
    const pm::TransformError err = pm::transform_error(a, b, points);
    // wrap(3.0 − (−3.0)) = wrap(6.0) = 6.0 − 2π
    EXPECT_NEAR(err.yaw, std::fabs(6.0 - 2.0 * pm::kPi), 1e-12);
    const pm::TransformError err_empty =
        pm::transform_error(a, b, std::vector<std::array<double, 3>>{});
    EXPECT_NEAR(err_empty.yaw, err.yaw, 1e-12);
}

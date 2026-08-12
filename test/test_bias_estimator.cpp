/**
 * test_bias_estimator.cpp
 * BiasEstimator + pose_math 单测。
 * 覆盖详设 6.1: UT-03 4DoF 投影 / UT-04 收敛时间常数 / UT-05 限速 clamp /
 * UT-06 单帧野值 / UT-07 真实跳变 / UT-08 双解翻转 / UT-09 断流与恢复 /
 * UT-10 初始化一致性 / UT-11 yaw 环绕 / UT-12 reset 位置补偿 /
 * UT-13 reset 航向补偿 / UT-14 (估计器部分: reset 清空候选)。
 * 附加: pose_math 恒等式 / reset 保 divergence / reset 后观测自洽。
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "map_odom_bias/core/bias_estimator.hpp"
#include "map_odom_bias/core/pose_math.hpp"

using map_odom_bias::BiasEstimator;
using map_odom_bias::BiasEstimatorParams;
using map_odom_bias::BiasState;
using map_odom_bias::HostTime;
using map_odom_bias::SampleTime;
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

void expect_t4_near(const pm::Transform4D & a, const pm::Transform4D & b,
                    double tol)
{
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
    EXPECT_NEAR(pm::wrap_angle(a.yaw - b.yaw), 0.0, tol);
}

void expect_pose_near(const pm::Pose & a, const pm::Pose & b, double tol)
{
    EXPECT_NEAR(a.p[0], b.p[0], tol);
    EXPECT_NEAR(a.p[1], b.p[1], tol);
    EXPECT_NEAR(a.p[2], b.p[2], tol);
    EXPECT_NEAR(pm::wrap_angle(pm::yaw_from_quat(a.q) - pm::yaw_from_quat(b.q)),
                0.0, tol);
}

/// 通用 fixture: 时间轴 + 喂观测/tick 工具; 默认 timeout 拉大避免误入 STALE
/// (STALE 用例显式改回), 其余参数取详设 3.3 默认值
class BiasEstimatorTest : public ::testing::Test
{
protected:
    static constexpr double kObsDt = 1.0 / 30.0;    // 30 Hz 观测
    static constexpr double kTickDt = 0.02;         // 50 Hz tick

    void SetUp() override
    {
        params_ = BiasEstimatorParams{};
        params_.observation_timeout = 1e9;    // 默认不触发 STALE
    }

    void feed(BiasEstimator & e, const pm::Transform4D & obs, int n = 1)
    {
        for (int i = 0; i < n; ++i) {
            t_ += kObsDt;
            e.add_observation(obs, SampleTime{t_}, HostTime{t_});
        }
    }

    void init_to(BiasEstimator & e, const pm::Transform4D & tf)
    {
        feed(e, tf, params_.init_confirm_frames);
        ASSERT_EQ(e.state(), BiasState::TRACKING);
    }

    /// n 拍定时步进; 首拍若无基准则 dt=0 (纯建基准拍)
    void run_ticks(BiasEstimator & e, int n)
    {
        for (int i = 0; i < n; ++i) {
            t_ += kTickDt;
            e.tick(HostTime{t_});
        }
    }

    BiasEstimatorParams params_;
    double t_{0.0};
};

}  // namespace

// ================= pose_math 基础 =================

TEST(PoseMath, WrapAngleBoundaries)
{
    EXPECT_NEAR(pm::wrap_angle(0.0), 0.0, 1e-12);
    EXPECT_NEAR(pm::wrap_angle(0.1), 0.1, 1e-12);
    EXPECT_NEAR(pm::wrap_angle(pm::kPi), pm::kPi, 1e-12);         // π → π
    EXPECT_NEAR(pm::wrap_angle(-pm::kPi), pm::kPi, 1e-12);        // -π → π
    EXPECT_NEAR(pm::wrap_angle(3.0 * pm::kPi), pm::kPi, 1e-9);
    EXPECT_NEAR(pm::wrap_angle(-3.0 * pm::kPi), pm::kPi, 1e-9);
    EXPECT_NEAR(pm::wrap_angle(2.0 * pm::kPi + 0.3), 0.3, 1e-9);
}

TEST(PoseMath, ComposeInverseIsIdentity)
{
    const pm::Transform4D t = t4(1.7, -2.3, 0.8, 2.1);
    expect_t4_near(pm::compose(t, pm::inverse(t)), t4(0, 0, 0, 0), 1e-12);
    expect_t4_near(pm::compose(pm::inverse(t), t), t4(0, 0, 0, 0), 1e-12);
}

TEST(PoseMath, YawQuatRoundtrip)
{
    for (double y = -3.0; y <= 3.0; y += 0.37) {
        EXPECT_NEAR(pm::yaw_from_quat(pm::quat_from_yaw(y)), y, 1e-12);
    }
}

TEST(PoseMath, SlerpMidpointOfYaw)
{
    const pm::Quat a = pm::quat_from_yaw(0.0);
    const pm::Quat b = pm::quat_from_yaw(pm::kPi / 2.0);
    const pm::Quat m = pm::quat_slerp(a, b, 0.5);
    EXPECT_NEAR(pm::yaw_from_quat(m), pm::kPi / 4.0, 1e-9);
}

TEST(PoseMath, BiasObservationRecoversKnownTransform)
{
    // 真值 T_map_odom (纯 4DoF), odom 位姿纯 yaw → 观测应精确还原真值
    const pm::Transform4D t_true = t4(1.0, 2.0, 0.5, 0.3);
    pm::Pose odom_base;
    odom_base.p = {{3.0, -1.0, 1.2}};
    odom_base.q = pm::quat_from_yaw(0.7);
    const pm::Pose map_base = pm::apply_to_pose(t_true, odom_base);
    const pm::Transform4D obs = pm::bias_observation(map_base, odom_base);
    expect_t4_near(obs, t_true, 1e-12);
}

// ---- UT-03: 观测注入 roll/pitch 扰动, 4DoF 投影隔离 ----
TEST(PoseMath, UT03_RollPitchProjection)
{
    const pm::Transform4D t_true = t4(1.0, 2.0, 0.5, 0.3);
    // odom 位姿本身带 roll/pitch (真实飞行姿态): 两侧一致的 roll/pitch
    // 在 T_obs = T_mb·T_ob⁻¹ 中消去, 观测仍精确还原真值
    pm::Pose odom_base;
    odom_base.p = {{3.0, -1.0, 1.2}};
    const pm::Quat q_rp = pm::quat_mul(
        pm::quat_from_yaw(0.7),
        pm::quat_mul(pm::Quat{std::cos(0.1), 0.0, std::sin(0.1), 0.0},     // pitch 0.2
                     pm::Quat{std::cos(0.05), std::sin(0.05), 0.0, 0.0}));  // roll 0.1
    odom_base.q = q_rp;
    const pm::Pose map_base = pm::apply_to_pose(t_true, odom_base);
    const pm::Transform4D obs = pm::bias_observation(map_base, odom_base);
    expect_t4_near(obs, t_true, 1e-12);

    // 观测侧额外 roll 扰动 (PnP 噪声): yaw 提取仍稳定, roll/pitch 被丢弃
    pm::Pose map_disturbed = map_base;
    map_disturbed.q = pm::quat_mul(
        pm::Quat{std::cos(0.025), std::sin(0.025), 0.0, 0.0},    // roll 0.05 扰动
        map_base.q);
    const pm::Transform4D obs2 = pm::bias_observation(map_disturbed, odom_base);
    EXPECT_NEAR(obs2.yaw, t_true.yaw, 0.02);
    // Transform4D 结构性无 roll/pitch 通道 —— raw 的 roll/pitch 恒为零
}

// ================= 初始化 =================

TEST_F(BiasEstimatorTest, UT10_ConsistentFramesInitialize)
{
    BiasEstimator e(params_);
    EXPECT_EQ(e.state(), BiasState::UNINITIALIZED);
    feed(e, t4(1.0, 2.0, 0.5, 0.3), 1);
    EXPECT_EQ(e.state(), BiasState::INITIALIZING);
    feed(e, t4(1.0, 2.0, 0.5, 0.3), 2);
    EXPECT_EQ(e.state(), BiasState::TRACKING);
    expect_t4_near(e.raw(), t4(1.0, 2.0, 0.5, 0.3), 1e-12);
    expect_t4_near(e.ctrl(), e.raw(), 1e-12);    // 初始化不走慢吸收
}

TEST_F(BiasEstimatorTest, UT10_InconsistentFramesStayInitializing)
{
    BiasEstimator e(params_);
    feed(e, t4(0.0, 0.0, 0.0, 0.0), 2);
    feed(e, t4(0.2, 0.0, 0.0, 0.0), 1);    // 偏 0.2 m > 半门限 0.15 → 候选重置
    EXPECT_EQ(e.state(), BiasState::INITIALIZING);
    feed(e, t4(0.2, 0.0, 0.0, 0.0), 1);
    EXPECT_EQ(e.state(), BiasState::INITIALIZING);    // 新起点已积 2 帧
    feed(e, t4(0.2, 0.0, 0.0, 0.0), 1);
    EXPECT_EQ(e.state(), BiasState::TRACKING);        // 满 3 帧
    expect_t4_near(e.raw(), t4(0.2, 0.0, 0.0, 0.0), 1e-12);
}

TEST_F(BiasEstimatorTest, InitUsesCandidateMean)
{
    BiasEstimator e(params_);
    feed(e, t4(0.00, 0.0, 0.0, 0.00), 1);
    feed(e, t4(0.03, 0.0, 0.0, 0.03), 1);
    feed(e, t4(0.06, 0.0, 0.0, 0.06), 1);
    ASSERT_EQ(e.state(), BiasState::TRACKING);
    expect_t4_near(e.raw(), t4(0.03, 0.0, 0.0, 0.03), 1e-12);
}

// ================= 门控 =================

TEST_F(BiasEstimatorTest, UT06_SingleOutlierRejected)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(2.0, 0.0, 0.0, 0.0), 1);    // 单帧野值偏 2 m → 入候选队列
    expect_t4_near(e.raw(), t4(0, 0, 0, 0), 1e-12);     // raw 不动
    expect_t4_near(e.ctrl(), t4(0, 0, 0, 0), 1e-12);    // ctrl 不动
    feed(e, t4(0.01, 0.0, 0.0, 0.0), 1);    // 回归正常 → 正常路径采纳,
    expect_t4_near(e.raw(), t4(0.01, 0, 0, 0), 1e-12);  // 野值候选被抛弃
    EXPECT_EQ(e.gate_reject_count(), 1u);    // 拒绝计数 +1 (被抛弃的野值)
    EXPECT_EQ(e.jump_count(), 0u);
}

TEST_F(BiasEstimatorTest, UT07_ConfirmedJumpAppliedOnceCtrlFollowsSlowly)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(0.5, 0.0, 0.0, 0.0), 2);
    expect_t4_near(e.raw(), t4(0, 0, 0, 0), 1e-12);    // 未满确认帧数, 账本不动
    feed(e, t4(0.5, 0.0, 0.0, 0.0), 1);                // 第 3 帧 → 确认
    expect_t4_near(e.raw(), t4(0.5, 0, 0, 0), 1e-12);  // 一次性跳变
    expect_t4_near(e.ctrl(), t4(0, 0, 0, 0), 1e-12);   // ctrl 不跳
    EXPECT_EQ(e.jump_count(), 1u);
    EXPECT_NEAR(e.last_jump_trans(), 0.5, 1e-12);
    // 确认为真实跳变的候选帧不计入拒绝 —— 合法重定位不污染误检信号
    EXPECT_EQ(e.gate_reject_count(), 0u);

    run_ticks(e, 51);    // 首拍建基准 + 50 拍 (1 s)
    // ctrl 按 τ=5s 慢速跟随: 1s ≈ 0.5·(1−e^{-0.2}) ≈ 0.0906
    EXPECT_GT(e.ctrl().x, 0.05);
    EXPECT_LT(e.ctrl().x, 0.15);
}

TEST_F(BiasEstimatorTest, UT08_FlipFlopYawNoOscillation)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    const double yaw30 = 30.0 * pm::kPi / 180.0;
    uint32_t last_reject = e.gate_reject_count();
    for (int i = 0; i < 20; ++i) {
        feed(e, t4(0.0, 0.0, 0.0, (i % 2 == 0) ? yaw30 : -yaw30), 1);
        EXPECT_NEAR(e.raw().yaw, 0.0, 1e-12);    // 账本不振荡
        if (i > 0) {
            // 每帧与前一候选不一致 → 前一候选被抛弃, 拒绝计数持续增长
            EXPECT_GT(e.gate_reject_count(), last_reject);
        }
        last_reject = e.gate_reject_count();
    }
    // 20 帧翻转: 前 19 帧候选各被下一帧抛弃 (末帧仍挂在队列中未定罪)
    EXPECT_EQ(e.gate_reject_count(), 19u);
    EXPECT_EQ(e.jump_count(), 0u);
}

TEST_F(BiasEstimatorTest, GateAcceptsWithinThreshold)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(0.29, 0.0, 0.0, 0.15), 1);    // 平移与 yaw 均在门限内
    expect_t4_near(e.raw(), t4(0.29, 0.0, 0.0, 0.15), 1e-12);
    EXPECT_EQ(e.gate_reject_count(), 0u);
}

// ================= 慢吸收 =================

TEST_F(BiasEstimatorTest, UT04_AbsorbTimeConstant)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(1.0, 0.0, 0.0, 0.0), 3);    // 确认跳变: raw 阶跃 1 m
    ASSERT_NEAR(e.raw().x, 1.0, 1e-12);
    run_ticks(e, 251);    // 首拍建基准 + 250 拍 × 0.02 s = 5.00 s = τ
    // 一阶低通 τ 时刻收敛 63.2% (离散步进 (1-dt/τ)^250 ≈ e⁻¹ 偏差 <0.1%)
    EXPECT_NEAR(e.ctrl().x, 0.632, 0.02);
    EXPECT_NEAR(e.raw().x, 1.0, 1e-12);    // raw 不受吸收影响
}

TEST_F(BiasEstimatorTest, UT05_RateClampOnLargeStep)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(10.0, 0.0, 0.0, 0.0), 3);    // 确认跳变: raw 阶跃 10 m
    ASSERT_NEAR(e.raw().x, 10.0, 1e-12);
    run_ticks(e, 1);    // 建基准
    double prev = e.ctrl().x;
    for (int i = 0; i < 100; ++i) {
        run_ticks(e, 1);
        const double step = e.ctrl().x - prev;
        // 每拍位移 ≤ max_correction_rate_trans · dt
        EXPECT_LE(step, params_.max_correction_rate_trans * kTickDt + 1e-9);
        prev = e.ctrl().x;
    }
    // clamp 激活期为匀速修正: 100 拍 × 0.004 m = 0.4 m
    EXPECT_NEAR(e.ctrl().x, 0.4, 1e-6);
}

TEST_F(BiasEstimatorTest, UT11_YawWrapShortestPath)
{
    BiasEstimator e(params_);
    const double yaw178 = 178.0 * pm::kPi / 180.0;
    init_to(e, t4(0, 0, 0, yaw178));
    // 观测 -178°: 与 +178° 最短角差 4°, 在门限内 → 正常采纳
    feed(e, t4(0.0, 0.0, 0.0, -yaw178), 1);
    EXPECT_NEAR(e.raw().yaw, -yaw178, 1e-12);
    ASSERT_NEAR(e.divergence_yaw(), 4.0 * pm::kPi / 180.0, 1e-9);

    // 吸收全程走最短路径 (跨 ±180° 边界), 偏差单调不增且不超 4°
    double prev_div = e.divergence_yaw();
    run_ticks(e, 1);
    for (int i = 0; i < 1000; ++i) {
        run_ticks(e, 1);
        EXPECT_LE(e.divergence_yaw(), prev_div + 1e-12);
        prev_div = e.divergence_yaw();
    }
    // 20 s ≈ 4τ: 残差 ≈ 4°·e⁻⁴ ≈ 0.13°
    EXPECT_NEAR(pm::wrap_angle(e.ctrl().yaw - (-yaw178)), 0.0, 0.01);
}

// ================= 断流与恢复 =================

// 冷启动只走时钟不喂观测: 状态恒 UNINITIALIZED, 不得进入 STALE
// (STALE ∈ initialized, 误入会导致薄壳未初始化就发布 identity ctrl;
//  bag 回放曾疑此路径, 实为回放脚本进程泄漏串台, 此用例把工况钉死)
TEST_F(BiasEstimatorTest, ColdStartTickOnlyStaysUninitialized)
{
    params_.observation_timeout = 2.0;
    BiasEstimator e(params_);
    run_ticks(e, 500);    // 10 s 纯 tick, 远超 observation_timeout
    EXPECT_EQ(e.state(), BiasState::UNINITIALIZED);
    EXPECT_FALSE(e.has_observation());
}

TEST_F(BiasEstimatorTest, UT09_StaleFreezeAndGatedRecovery)
{
    params_.observation_timeout = 2.0;
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(0.5, 0.0, 0.0, 0.0), 3);    // raw 跳到 0.5
    run_ticks(e, 51);                      // 1 s 吸收
    ASSERT_EQ(e.state(), BiasState::TRACKING);
    ASSERT_GT(e.ctrl().x, 0.05);

    run_ticks(e, 75);    // 观测停止, 累计 2.5 s > timeout
    EXPECT_EQ(e.state(), BiasState::STALE);
    const double frozen_x = e.ctrl().x;
    run_ticks(e, 50);    // STALE 冻结: ctrl 停止收敛步进
    EXPECT_EQ(e.ctrl().x, frozen_x);
    expect_t4_near(e.raw(), t4(0.5, 0, 0, 0), 1e-12);    // raw 保持

    // 恢复观测带偏差 0.4 (> 门限): 走门控候选路径
    feed(e, t4(0.9, 0.0, 0.0, 0.0), 1);
    EXPECT_EQ(e.state(), BiasState::STALE);    // 未确认前不回 TRACKING
    feed(e, t4(0.9, 0.0, 0.0, 0.0), 2);
    EXPECT_EQ(e.state(), BiasState::TRACKING);    // 确认后恢复
    expect_t4_near(e.raw(), t4(0.9, 0, 0, 0), 1e-12);
    EXPECT_EQ(e.jump_count(), 2u);    // 0→0.5 与 0.5→0.9 两次确认跳变
}

TEST_F(BiasEstimatorTest, StaleRecoveryWithinGateIsImmediate)
{
    params_.observation_timeout = 2.0;
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    run_ticks(e, 126);    // 2.52 s 无观测 → STALE
    ASSERT_EQ(e.state(), BiasState::STALE);
    feed(e, t4(0.1, 0.0, 0.0, 0.0), 1);    // 门限内 → 直接采纳
    EXPECT_EQ(e.state(), BiasState::TRACKING);
    expect_t4_near(e.raw(), t4(0.1, 0, 0, 0), 1e-12);
}

// ================= EKF reset 瞬跳补偿 =================

TEST_F(BiasEstimatorTest, UT12_PositionResetCompensation)
{
    BiasEstimator e(params_);
    init_to(e, t4(1.0, 2.0, 0.5, 0.3));
    feed(e, t4(1.05, 2.0, 0.5, 0.3), 1);    // 拉开 raw 与 ctrl (门限内采纳)
    const uint32_t reject_before = e.gate_reject_count();

    pm::Pose odom_old;
    odom_old.p = {{3.0, -1.0, 1.2}};
    odom_old.q = pm::quat_from_yaw(0.7);
    const pm::Pose map_before_raw = pm::apply_to_pose(e.raw(), odom_old);
    const pm::Pose map_before_ctrl = pm::apply_to_pose(e.ctrl(), odom_old);

    // 位置 reset: odom 系原点改写, 机体 odom 坐标 +Δp
    const pm::Transform4D d = t4(0.4, -0.2, 0.1, 0.0);
    e.apply_reset(d);

    pm::Pose odom_new = odom_old;
    odom_new.p[0] += d.x;
    odom_new.p[1] += d.y;
    odom_new.p[2] += d.z;
    // 补偿前后机体 map 系位姿逐位连续
    expect_pose_near(pm::apply_to_pose(e.raw(), odom_new), map_before_raw, 1e-12);
    expect_pose_near(pm::apply_to_pose(e.ctrl(), odom_new), map_before_ctrl, 1e-12);
    EXPECT_EQ(e.gate_reject_count(), reject_before);    // 门控拒绝计数不增
    EXPECT_EQ(e.reset_event_count(), 1u);
    EXPECT_NEAR(e.last_reset_trans(), std::sqrt(0.4 * 0.4 + 0.2 * 0.2 + 0.1 * 0.1),
                1e-12);
}

TEST_F(BiasEstimatorTest, UT13_HeadingResetCompensation)
{
    BiasEstimator e(params_);
    init_to(e, t4(1.0, 2.0, 0.5, 0.3));

    // 机体不在 odom 原点
    const std::array<double, 3> p_ob = {{2.0, 1.0, 0.8}};
    pm::Pose odom_old;
    odom_old.p = p_ob;
    odom_old.q = pm::quat_from_yaw(0.5);
    const pm::Pose map_before = pm::apply_to_pose(e.raw(), odom_old);
    const pm::Pose map_before_ctrl = pm::apply_to_pose(e.ctrl(), odom_old);

    // 航向 reset: odom 系绕机体当前位置旋转 Δψ (位置坐标不动)
    //   D = [Rz(Δψ), (I−Rz(Δψ))·p_ob]
    const double dpsi = 0.2;
    const double c = std::cos(dpsi);
    const double s = std::sin(dpsi);
    const pm::Transform4D d = t4(
        p_ob[0] - (c * p_ob[0] - s * p_ob[1]),
        p_ob[1] - (s * p_ob[0] + c * p_ob[1]),
        0.0, dpsi);
    e.apply_reset(d);

    // reset 后机体 odom 位姿: 位置不变 (绕自身旋转), yaw + Δψ
    pm::Pose odom_new;
    odom_new.p = p_ob;
    odom_new.q = pm::quat_from_yaw(0.5 + dpsi);
    expect_pose_near(pm::apply_to_pose(e.raw(), odom_new), map_before, 1e-12);
    expect_pose_near(pm::apply_to_pose(e.ctrl(), odom_new), map_before_ctrl, 1e-12);
    // raw/ctrl 为 Transform4D, roll/pitch 结构性恒零 (4DoF 保持)
    EXPECT_NEAR(e.raw().yaw, pm::wrap_angle(0.3 - dpsi), 1e-12);
}

TEST_F(BiasEstimatorTest, UT14_ResetClearsGateCandidates)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(0.5, 0.0, 0.0, 0.0), 2);    // 候选累积 2 帧 (还差 1 帧确认)
    e.apply_reset(t4(0.2, 0.0, 0.0, 0.0));    // 候选是旧坐标系偏差 → 作废

    feed(e, t4(0.5, 0.0, 0.0, 0.0), 1);
    EXPECT_EQ(e.jump_count(), 0u);    // 候选已清空, 该帧是新起点而非第 3 帧
    feed(e, t4(0.5, 0.0, 0.0, 0.0), 2);
    EXPECT_EQ(e.jump_count(), 1u);    // 重新累积满 3 帧后功能正常
}

TEST_F(BiasEstimatorTest, ResetPreservesRawCtrlDivergence)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(0.05, 0.02, 0.0, 0.0), 1);    // raw 移动, ctrl 不动 (yaw 相同)
    const double div_t = e.divergence_trans();
    const double div_y = e.divergence_yaw();
    ASSERT_GT(div_t, 0.0);

    // 带 yaw 的 reset: raw/ctrl 施加同一 D⁻¹, yaw 无滞后时坐标差严格不变
    e.apply_reset(t4(0.3, -0.1, 0.2, 0.15));
    EXPECT_NEAR(e.divergence_trans(), div_t, 1e-12);
    EXPECT_NEAR(e.divergence_yaw(), div_y, 1e-12);
}

TEST_F(BiasEstimatorTest, ResetDuringInitializingRestartsCandidates)
{
    BiasEstimator e(params_);
    feed(e, t4(0, 0, 0, 0), 2);    // 初始化候选 2 帧
    ASSERT_EQ(e.state(), BiasState::INITIALIZING);
    e.apply_reset(t4(0.5, 0.0, 0.0, 0.0));
    EXPECT_EQ(e.state(), BiasState::INITIALIZING);
    feed(e, t4(0.1, 0.0, 0.0, 0.0), 1);    // 新坐标系首帧成为新起点
    EXPECT_EQ(e.state(), BiasState::INITIALIZING);
    feed(e, t4(0.1, 0.0, 0.0, 0.0), 2);
    EXPECT_EQ(e.state(), BiasState::TRACKING);    // 重新累积满 3 帧
    expect_t4_near(e.raw(), t4(0.1, 0, 0, 0), 1e-12);
}

TEST_F(BiasEstimatorTest, ResetThenConsistentObservationPassesGate)
{
    BiasEstimator e(params_);
    init_to(e, t4(1.0, 0.0, 0.0, 0.0));
    // 物理连续场景: odom 原点平移 0.5, 补偿后账本 raw = 1.0 − 0.5 = 0.5;
    // 新坐标系下的偏差观测正是 0.5 → 应走正常路径, 不被误判突变
    e.apply_reset(t4(0.5, 0.0, 0.0, 0.0));
    expect_t4_near(e.raw(), t4(0.5, 0, 0, 0), 1e-12);
    feed(e, t4(0.5, 0.0, 0.0, 0.0), 1);
    EXPECT_EQ(e.gate_reject_count(), 0u);
    EXPECT_EQ(e.state(), BiasState::TRACKING);
}

// ================= 对抗复核补强用例 (F01/F02/F06/F12/F17-F21) =================

// ---- F01: 大力臂 + 纯 pitch 噪声, 平移观测不被 roll/pitch 污染 ----
TEST(PoseMath, F01_PitchNoiseDoesNotLeakIntoTranslation)
{
    const pm::Transform4D t_true = t4(1.0, 2.0, 0.5, 0.3);
    pm::Pose odom_base;
    odom_base.p = {{6.0, 0.0, 1.5}};    // 力臂 6 m (复核 probe 场景)
    odom_base.q = pm::quat_from_yaw(0.7);
    pm::Pose map_base = pm::apply_to_pose(t_true, odom_base);
    // PnP 观测侧 pitch 噪声 2° (世界系左乘): 完整-R 反解会在 z 注入
    // ≈ 6·sin(2°) ≈ 0.21 m; yaw-first 投影应把泄漏压到 mm 级
    const double half = 2.0 * pm::kPi / 180.0 / 2.0;
    map_base.q = pm::quat_mul(
        pm::Quat{std::cos(half), 0.0, std::sin(half), 0.0}, map_base.q);
    const pm::Transform4D obs = pm::bias_observation(map_base, odom_base);
    EXPECT_NEAR(obs.z, t_true.z, 0.01);     // 旧实现此处误差 ~0.21 m
    EXPECT_NEAR(obs.x, t_true.x, 0.05);
    EXPECT_NEAR(obs.y, t_true.y, 0.05);
    EXPECT_NEAR(obs.yaw, t_true.yaw, 0.01);
}

// ---- F06: 航向 reset D 构造 (旋转中心/符号/合成序) ----
TEST(PoseMath, F06_HeadingResetDeltaConstruction)
{
    const double dpsi = 0.2;
    const std::array<double, 3> p_ob = {{2.0, 1.0, 0.8}};
    const pm::Transform4D d = pm::make_heading_reset_delta(dpsi, p_ob);
    EXPECT_NEAR(d.yaw, dpsi, 1e-12);
    // 旋转中心不变性: 机体当前位置坐标不动
    const auto p_center = pm::apply(d, p_ob);
    EXPECT_NEAR(p_center[0], p_ob[0], 1e-12);
    EXPECT_NEAR(p_center[1], p_ob[1], 1e-12);
    EXPECT_NEAR(p_center[2], p_ob[2], 1e-12);
    // 符号: 中心以东 1 m 的点绕中心逆时针 (ENU yaw CCW 正) 旋转 dpsi
    const auto p_east = pm::apply(d, {{p_ob[0] + 1.0, p_ob[1], p_ob[2]}});
    EXPECT_NEAR(p_east[0], p_ob[0] + std::cos(dpsi), 1e-12);
    EXPECT_NEAR(p_east[1], p_ob[1] + std::sin(dpsi), 1e-12);
    // 同帧并发合成序: D_total = D_yaw·D_pos 等价于先位置后航向依次施加
    const pm::Transform4D d_pos = t4(0.4, -0.2, 0.1, 0.0);
    const pm::Transform4D d_total = pm::compose(d, d_pos);
    const std::array<double, 3> p = {{-1.0, 3.0, 0.5}};
    const auto seq = pm::apply(d, pm::apply(d_pos, p));
    const auto tot = pm::apply(d_total, p);
    EXPECT_NEAR(tot[0], seq[0], 1e-12);
    EXPECT_NEAR(tot[1], seq[1], 1e-12);
    EXPECT_NEAR(tot[2], seq[2], 1e-12);
}

// ---- F19: slerp 双倍覆盖 (dot<0 翻转取最短路径) ----
TEST(PoseMath, F19_SlerpNegatedQuaternionShortestPath)
{
    const pm::Quat a = pm::quat_from_yaw(0.0);
    const pm::Quat b_pos = pm::quat_from_yaw(pm::kPi / 2.0);
    const pm::Quat b_neg{-b_pos.w, -b_pos.x, -b_pos.y, -b_pos.z};    // 同旋转
    const pm::Quat m = pm::quat_slerp(a, b_neg, 0.5);
    EXPECT_NEAR(pm::yaw_from_quat(m), pm::kPi / 4.0, 1e-9);    // 走短弧
}

// ---- F02: 误检风暴下账本停更 → 进 STALE 告警 ----
TEST_F(BiasEstimatorTest, F02_RejectStormEntersStale)
{
    params_.observation_timeout = 2.0;
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    // 3 s 风暴: 观测持续到达但全部被门控拒绝 (交替 ±2 m 永不累积确认)
    for (int i = 0; i < 15; ++i) {
        feed(e, t4((i % 2 == 0) ? 2.0 : -2.0, 0.0, 0.0, 0.0), 1);
        run_ticks(e, 10);    // 0.2 s / 轮
    }
    // 被拒观测不算"有效": 账本停更超时应进 STALE (而非伪装健康)
    EXPECT_EQ(e.state(), BiasState::STALE);
    expect_t4_near(e.raw(), t4(0, 0, 0, 0), 1e-12);    // 账本冻结
    EXPECT_GT(e.gate_reject_count(), 10u);
    // 风暴中的野值仍不能把状态拉回 TRACKING
    feed(e, t4(2.0, 0.0, 0.0, 0.0), 1);
    EXPECT_EQ(e.state(), BiasState::STALE);
}

// ---- F12: 非有限观测在入口被拦截, 账本免疫 ----
TEST_F(BiasEstimatorTest, F12_NonFiniteObservationRejected)
{
    BiasEstimator e(params_);
    init_to(e, t4(0.1, 0.0, 0.0, 0.0));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    feed(e, t4(nan, 0.0, 0.0, 0.0), 1);
    feed(e, t4(0.0, 0.0, 0.0, std::numeric_limits<double>::infinity()), 1);
    expect_t4_near(e.raw(), t4(0.1, 0, 0, 0), 1e-12);    // 账本不被污染
    EXPECT_EQ(e.state(), BiasState::TRACKING);
    EXPECT_EQ(e.invalid_obs_count(), 2u);
    EXPECT_EQ(e.gate_reject_count(), 0u);    // 与门控拒绝分开计数
    // 后续正常观测不受影响
    feed(e, t4(0.12, 0.0, 0.0, 0.0), 1);
    expect_t4_near(e.raw(), t4(0.12, 0, 0, 0), 1e-12);
}

// ---- F17: 候选队列帧间一致性用门限之半 (而非整门限) ----
TEST_F(BiasEstimatorTest, F17_CandidateConsistencyUsesHalfThreshold)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(0.50, 0.0, 0.0, 0.0), 1);    // 突变帧 A 入队
    // B 与 A 差 0.20: 在整门限 (0.3) 内但超半门限 (0.15) → 必须重置而非累积
    feed(e, t4(0.70, 0.0, 0.0, 0.0), 1);
    EXPECT_EQ(e.gate_reject_count(), 1u);    // A 被抛弃
    feed(e, t4(0.70, 0.0, 0.0, 0.0), 2);     // B 起新队列, 满 3 帧确认
    EXPECT_EQ(e.jump_count(), 1u);
    // raw = 新候选均值 0.70; 若误用整门限会混入 A → mean(0.5,0.7,0.7)=0.633
    expect_t4_near(e.raw(), t4(0.70, 0, 0, 0), 1e-12);
}

// ---- F18: 门控 within 与慢吸收 clamp 的向量范数口径 ----
TEST_F(BiasEstimatorTest, F18_GateUsesVectorNorm)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    // 逐轴均 < 0.3 但范数 0.32 > 0.3 → 应按范数判超限入候选
    feed(e, t4(0.25, 0.20, 0.0, 0.0), 1);
    expect_t4_near(e.raw(), t4(0, 0, 0, 0), 1e-12);    // 未被采纳
    // 范数 0.29 < 0.3 → 采纳
    feed(e, t4(0.20, 0.21, 0.0, 0.0), 1);
    expect_t4_near(e.raw(), t4(0.20, 0.21, 0, 0), 1e-12);
}

TEST_F(BiasEstimatorTest, F18_AbsorbClampPreservesDirection)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(3.0, 4.0, 0.0, 0.0), 3);    // 确认跳变: 模 5 m, 方向 (0.6, 0.8)
    ASSERT_NEAR(e.raw().x, 3.0, 1e-12);
    run_ticks(e, 51);    // 建基准 + 1 s 满速修正 (clamp 激活)
    // 范数限幅保方向: 步进向量 ∝ (0.6, 0.8), 1 s 累计模 0.2 m
    const double norm = std::sqrt(e.ctrl().x * e.ctrl().x +
                                  e.ctrl().y * e.ctrl().y);
    EXPECT_NEAR(norm, 0.2, 1e-6);
    EXPECT_NEAR(e.ctrl().y / e.ctrl().x, 4.0 / 3.0, 1e-9);    // 逐轴 clamp 则 1:1
}

// ---- F19: max_tick_dt 单拍防御 (定时器长挂起恢复不产生大跳) ----
TEST_F(BiasEstimatorTest, F19_MaxTickDtLimitsSingleStep)
{
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(1.0, 0.0, 0.0, 0.0), 3);    // raw 跳到 1 m
    run_ticks(e, 1);                        // 建基准
    e.tick(HostTime{t_ + 10.0});                      // 10 s 空窗后单拍
    // dt 被截到 max_tick_dt=0.2: step = 1·(0.2/5) = 0.04 m (不限则一步到位)
    EXPECT_LE(e.ctrl().x, 0.05);
    EXPECT_GT(e.ctrl().x, 0.01);
}

// ---- F20: 候选均值的防环绕 yaw (±180° 附近初始化不塌缩) ----
TEST_F(BiasEstimatorTest, F20_CandidateMeanYawWrapSafe)
{
    BiasEstimator e(params_);
    const double d179 = 179.0 * pm::kPi / 180.0;
    const double dm1795 = -179.5 * pm::kPi / 180.0;
    const double d17975 = 179.75 * pm::kPi / 180.0;
    feed(e, t4(0, 0, 0, d179), 1);
    feed(e, t4(0, 0, 0, dm1795), 1);     // 与前帧最短角差 1.5° (一致)
    feed(e, t4(0, 0, 0, d17975), 1);     // 差 1.25° (一致) → 初始化完成
    ASSERT_EQ(e.state(), BiasState::TRACKING);
    // 相对角均值 ≈ 179.75°; 算术平均会塌缩到 ~59.7° (0.63π/3)
    EXPECT_GT(std::fabs(e.raw().yaw), 3.10);
}

// ---- F21: raw 中值去毛刺窗口 (raw_median_window=3) ----
TEST_F(BiasEstimatorTest, F21_RawMedianWindowFiltersSpike)
{
    params_.raw_median_window = 3;
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    feed(e, t4(0.0, 0.0, 0.0, 0.0), 1);
    // 单帧毛刺 (门限内, 会被正常路径采纳) 被中值窗口滤除
    feed(e, t4(0.05, 0.0, 0.0, 0.0), 1);
    EXPECT_NEAR(e.raw().x, 0.0, 1e-12);    // window {0, 0, 0.05} 中值 = 0
    feed(e, t4(0.0, 0.0, 0.0, 0.0), 1);
    EXPECT_NEAR(e.raw().x, 0.0, 1e-12);
    // 持续的真实变化两帧后进账 (中值滤波的固有延迟)
    feed(e, t4(0.05, 0.0, 0.0, 0.0), 1);
    feed(e, t4(0.05, 0.0, 0.0, 0.0), 1);
    EXPECT_NEAR(e.raw().x, 0.05, 1e-12);
}

TEST_F(BiasEstimatorTest, ResetInStaleStillCompensates)
{
    params_.observation_timeout = 2.0;
    BiasEstimator e(params_);
    init_to(e, t4(1.0, 0.0, 0.0, 0.0));
    run_ticks(e, 126);    // → STALE
    ASSERT_EQ(e.state(), BiasState::STALE);

    // STALE 也执行补偿 —— 否则断流恢复时 raw 与首帧观测相差整个 delta
    e.apply_reset(t4(0.5, 0.0, 0.0, 0.0));
    expect_t4_near(e.raw(), t4(0.5, 0, 0, 0), 1e-12);
    EXPECT_EQ(e.state(), BiasState::STALE);
    // 恢复首帧与补偿后的 raw 一致 → 直接采纳回 TRACKING
    feed(e, t4(0.5, 0.0, 0.0, 0.0), 1);
    EXPECT_EQ(e.state(), BiasState::TRACKING);
    EXPECT_EQ(e.gate_reject_count(), 0u);
}

// ================= cmd 快通道 (B 方案出口) =================

TEST_F(BiasEstimatorTest, CmdInitializedEqualToRawAndCtrl)
{
    // 初始化不走吸收: raw = ctrl = cmd = 候选均值 (非零 T0 防假绿)
    BiasEstimator e(params_);
    init_to(e, t4(1.0, 2.0, 0.5, 0.3));
    expect_t4_near(e.cmd(), e.raw(), 1e-12);
    expect_t4_near(e.cmd(), e.ctrl(), 1e-12);
    EXPECT_NEAR(e.divergence_cmd_trans(), 0.0, 1e-12);
    EXPECT_NEAR(e.divergence_cmd_yaw(), 0.0, 1e-12);
}

TEST_F(BiasEstimatorTest, CmdConvergesFasterThanCtrl)
{
    // 同一 raw 阶跃, cmd (τ_f=0.5) 一秒内基本收敛, ctrl (τ=5) 仍挂大半欠账
    // —— B 的机制本体: 出口贴 raw, 入口保持稳
    BiasEstimator e(params_);
    init_to(e, t4(0.0, 0.0, 0.0, 0.0));
    run_ticks(e, 1);                          // 建 tick 基准
    feed(e, t4(0.2, 0.0, 0.0, 0.0), 1);       // 门限内直接采纳, raw 阶跃 0.2
    run_ticks(e, 50);                         // ~1.03 s
    const double cmd_residual = std::fabs(e.raw().x - e.cmd().x);
    const double ctrl_residual = std::fabs(e.raw().x - e.ctrl().x);
    EXPECT_LT(cmd_residual, 0.05);            // e^{-2} 量级
    EXPECT_GT(ctrl_residual, 0.14);           // e^{-0.2} 量级
    EXPECT_GT(ctrl_residual, cmd_residual * 3.0);
    // divergence_cmd = |cmd − ctrl| 自洽对账
    EXPECT_NEAR(e.divergence_cmd_trans(),
                std::fabs(e.cmd().x - e.ctrl().x), 1e-12);
}

TEST_F(BiasEstimatorTest, CmdRateClampIndependentOfCtrl)
{
    // 跳变确认后 raw 一次跳 2.0 m, 单拍步进受各自钳位:
    // cmd ≤ cmd_max_correction_rate_trans·dt, ctrl ≤ max_correction_rate_trans·dt
    BiasEstimator e(params_);
    init_to(e, t4(0.0, 0.0, 0.0, 0.0));
    run_ticks(e, 1);                          // 建 tick 基准
    feed(e, t4(2.0, 0.0, 0.0, 0.0), 3);       // 候选路径 3 帧确认, raw → 2.0
    ASSERT_NEAR(e.raw().x, 2.0, 1e-12);
    const double cmd_before = e.cmd().x;
    const double ctrl_before = e.ctrl().x;
    // dt = 3 帧观测间隔 + 1 tick = 3/30 + 0.02 = 0.12 s;
    // 无钳位步进 = 2.0·dt/τ (cmd 0.48 / ctrl 0.048) 均超各自限幅 → 取限幅
    run_ticks(e, 1);
    EXPECT_NEAR(e.cmd().x - cmd_before,
                params_.cmd_max_correction_rate_trans * 0.12, 1e-9);
    EXPECT_NEAR(e.ctrl().x - ctrl_before,
                params_.max_correction_rate_trans * 0.12, 1e-9);
}

TEST_F(BiasEstimatorTest, CmdFrozenInStale)
{
    // STALE 双通道冻结: cmd 与 ctrl 一样保持最后估计 (断流不外推)
    params_.observation_timeout = 2.0;
    BiasEstimator e(params_);
    init_to(e, t4(0.0, 0.0, 0.0, 0.0));
    feed(e, t4(0.1, 0.0, 0.0, 0.0), 1);       // 拉开 raw
    run_ticks(e, 126);                        // 2.52 s 无观测 → STALE
    ASSERT_EQ(e.state(), BiasState::STALE);
    const pm::Transform4D cmd_at_stale = e.cmd();
    run_ticks(e, 50);
    expect_t4_near(e.cmd(), cmd_at_stale, 1e-12);
}

TEST_F(BiasEstimatorTest, CmdResetCompensatedWithRawAndCtrl)
{
    // 同一 D 三状态一致补偿: 机体 map 系位姿跨 reset 连续 (UT-12 同款),
    // cmd−ctrl 分歧 (快慢通道欠账差) 跨 reset 保持
    BiasEstimator e(params_);
    init_to(e, t4(1.0, 2.0, 0.5, 0.3));
    run_ticks(e, 1);
    feed(e, t4(1.1, 2.0, 0.5, 0.3), 1);       // 拉开 raw
    run_ticks(e, 10);                         // cmd 吸收快于 ctrl → cmd ≠ ctrl
    const double div_before = e.divergence_cmd_trans();
    ASSERT_GT(div_before, 1e-4);

    pm::Pose odom_old;
    odom_old.p = {{3.0, -1.0, 1.2}};
    odom_old.q = pm::quat_from_yaw(0.7);
    const pm::Pose map_before_cmd = pm::apply_to_pose(e.cmd(), odom_old);

    const pm::Transform4D d = t4(0.4, -0.2, 0.1, 0.0);
    e.apply_reset(d);

    pm::Pose odom_new = odom_old;
    odom_new.p[0] += d.x;
    odom_new.p[1] += d.y;
    odom_new.p[2] += d.z;
    expect_pose_near(pm::apply_to_pose(e.cmd(), odom_new), map_before_cmd, 1e-12);
    EXPECT_NEAR(e.divergence_cmd_trans(), div_before, 1e-12);
}

// ---- ③度量空间修法: 机体点残差门控 / 钳位预算 / divergence ----
// (设计文档 v1 四节 4.3; 默认参数 = 原点评估 = 参数空间旧行为,
//  上面全部既有用例即修法的行为兼容回归)

TEST_F(BiasEstimatorTest, GateAtBodyPointYawNoiseWithLeverArmNotRejected)
{
    // 08-06 风暴工况: 力臂 r_horiz=5m, yaw 解算噪声 4° → 变换参数空间
    // 平移伪差 2·sin(2°)·5 ≈ 0.35m 超 0.3 门限 (旧度量整帧被拒);
    // 机体点残差恒零 + yaw 差 4° < 10° 门限 → 应正常采纳
    BiasEstimator e(params_);
    const pm::Transform4D t_true = t4(0.2, -0.1, 0.0, 0.15);
    const std::array<double, 3> p_ob = {{4.0, 3.0, 0.5}};    // r_horiz = 5
    for (int i = 0; i < params_.init_confirm_frames; ++i) {
        t_ += kObsDt;
        e.add_observation(t_true, SampleTime{t_}, HostTime{t_}, p_ob);
    }
    ASSERT_EQ(e.state(), BiasState::TRACKING);

    pm::Pose ob;
    ob.p = p_ob;
    ob.q = pm::quat_from_yaw(0.9);
    pm::Pose mb = pm::apply_to_pose(t_true, ob);
    mb.q = pm::quat_normalize(
        pm::quat_mul(pm::quat_from_yaw(4.0 * pm::kPi / 180.0), mb.q));
    const pm::Transform4D obs = pm::bias_observation(mb, ob);
    // 前置自检: 参数空间平移伪差确实超门限 (旧度量下该帧会被拒)
    ASSERT_GT(std::hypot(obs.x - t_true.x, obs.y - t_true.y),
              params_.gate_trans_threshold);

    t_ += kObsDt;
    e.add_observation(obs, SampleTime{t_}, HostTime{t_}, p_ob);
    EXPECT_EQ(e.gate_reject_count(), 0u);
    EXPECT_NEAR(e.raw().x, obs.x, 1e-12);    // 正常路径原值采纳
    EXPECT_NEAR(e.raw().yaw, obs.yaw, 1e-12);
}

TEST_F(BiasEstimatorTest, GateAtBodyPointTrueTranslationStillRejected)
{
    // 机体点度量不放走真平移野值: 0.5m 平移突变在任意力臂下仍超门限
    BiasEstimator e(params_);
    const std::array<double, 3> p_ob = {{4.0, 3.0, 0.5}};
    for (int i = 0; i < params_.init_confirm_frames; ++i) {
        t_ += kObsDt;
        e.add_observation(t4(0.0, 0.0, 0.0, 0.0), SampleTime{t_}, HostTime{t_}, p_ob);
    }
    ASSERT_EQ(e.state(), BiasState::TRACKING);
    t_ += kObsDt;
    e.add_observation(t4(0.5, 0.0, 0.0, 0.0), SampleTime{t_}, HostTime{t_}, p_ob);
    EXPECT_NEAR(e.raw().x, 0.0, 1e-12);    // 未入账本 (走候选队列)
}

TEST_F(BiasEstimatorTest, AbsorbClampBudgetAtBodyPoint)
{
    // raw 与 ctrl 差纯 yaw 0.1 rad, 力臂 10m: 参数空间旧钳位平移步进
    // 为零不触发, yaw 仅受 rate_yaw 限幅 → 机体点每拍被拉
    // ~0.17·0.02·10 = 0.034m 远超平移预算 0.2·0.02 = 0.004m;
    // 机体点预算钳位: setpoint 被拉动速率 ≤ max_correction_rate_trans
    BiasEstimator e(params_);
    init_to(e, t4(0.0, 0.0, 0.0, 0.0));
    feed(e, t4(0.0, 0.0, 0.0, 0.1), 1);    // yaw 0.1 < 门限 → 采纳
    ASSERT_NEAR(e.raw().yaw, 0.1, 1e-12);
    run_ticks(e, 1);    // 首拍建 tick 基准 (dt=0 无步进), 后续每拍 dt=kTickDt

    const std::vector<std::array<double, 3>> eval = {{{10.0, 0.0, 0.0}}};
    for (int i = 0; i < 50; ++i) {
        const pm::Transform4D before = e.ctrl();
        t_ += kTickDt;
        e.tick(HostTime{t_}, eval);
        const pm::TransformError moved =
            pm::transform_error(e.ctrl(), before, eval);
        EXPECT_LE(moved.trans,
                  params_.max_correction_rate_trans * kTickDt + 1e-9);
    }
    EXPECT_GT(e.ctrl().yaw, 0.0);    // 预算内仍在收敛, 非死锁
}

// ---- ⑤钟域: STALE 以到达刻判定 / 双钟记账 / iteration 计数 ----

TEST_F(BiasEstimatorTest, StaleJudgedByArrivalNotSampleStamp)
{
    // 深延迟链路 (采样戳滞后到达刻 2.5s > timeout 2.0s), 观测流本身健康:
    // 旧混域判定 (tick 到达刻 − 采样戳) 会把常驻延迟误判为断流;
    // ⑤修正后 STALE 只看到达刻年龄 → 保持 TRACKING
    params_.observation_timeout = 2.0;
    BiasEstimator e(params_);
    const double delay = 2.5;
    for (int i = 0; i < 3; ++i) {
        t_ += kObsDt;
        e.add_observation(t4(0, 0, 0, 0), SampleTime{t_ - delay}, HostTime{t_});
    }
    ASSERT_EQ(e.state(), BiasState::TRACKING);
    EXPECT_NEAR(e.last_obs_delay(), delay, 1e-12);    // 延迟常驻可观测
    // 观测持续到达 (到达间隔 << timeout), 其间 tick 不应判 STALE
    for (int i = 0; i < 60; ++i) {
        t_ += kObsDt;
        e.add_observation(t4(0, 0, 0, 0), SampleTime{t_ - delay}, HostTime{t_});
        t_ += kTickDt;
        e.tick(HostTime{t_});
        ASSERT_EQ(e.state(), BiasState::TRACKING) << "i=" << i;
    }
    // 真断流 (到达停止) 仍要判 STALE: 语义未被放松
    t_ += 2.1;
    e.tick(HostTime{t_});
    EXPECT_EQ(e.state(), BiasState::STALE);
}

TEST_F(BiasEstimatorTest, ReferenceIterationCountsJumpAndReset)
{
    // 参考系事件单调计数 (④ResetEvent 的 iteration 数据源):
    // 初始化不计, 跳变确认 +1, reset 补偿 +1
    BiasEstimator e(params_);
    init_to(e, t4(0, 0, 0, 0));
    EXPECT_EQ(e.reference_iteration(), 0u);
    feed(e, t4(1.0, 0.0, 0.0, 0.0), 3);       // 候选 3 帧确认跳变
    ASSERT_EQ(e.jump_count(), 1u);
    EXPECT_EQ(e.reference_iteration(), 1u);
    e.apply_reset(t4(0.2, 0.0, 0.0, 0.0));    // EKF reset 补偿
    ASSERT_EQ(e.reset_event_count(), 1u);
    EXPECT_EQ(e.reference_iteration(), 2u);
}

TEST_F(BiasEstimatorTest, DivergenceAtBodyPoint)
{
    // raw/ctrl 纯 yaw 分歧 0.1 rad 在 6m 力臂处 = 感知与指令位置分歧
    // 2·sin(0.05)·6; 参数空间口径 (无参默认) 看不见该分歧
    BiasEstimator e(params_);
    init_to(e, t4(0.0, 0.0, 0.0, 0.0));
    feed(e, t4(0.0, 0.0, 0.0, 0.1), 1);    // raw 更新, 无 tick 不吸收
    const std::vector<std::array<double, 3>> eval = {{{6.0, 0.0, 0.0}}};
    EXPECT_NEAR(e.divergence_trans(), 0.0, 1e-12);
    EXPECT_NEAR(e.divergence_trans(eval), 2.0 * std::sin(0.05) * 6.0, 1e-9);
    EXPECT_NEAR(e.divergence_yaw(), 0.1, 1e-12);
    EXPECT_NEAR(e.divergence_cmd_trans(eval), 0.0, 1e-12);    // cmd==ctrl
}

/**
 * test_odom_buffer.cpp
 * OdomBuffer 单测 —— UT-01 插值精度 / UT-02 缓冲边界 / UT-14 (缓冲部分:
 * reset 清空 + 静默窗口); 附加: 乱序防御 / 时长剔除 / 外推零阶保持 /
 * 精确命中 / 空缓冲计数。用例编号对应详设 6.1 节。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "map_odom_bias/core/odom_buffer.hpp"

using map_odom_bias::OdomBuffer;
using map_odom_bias::OdomSample;
using map_odom_bias::SampleTime;
namespace pm = map_odom_bias::pose_math;

namespace
{

// 解析轨迹: p = (sin(2πt), cos(2πt), 0.5·sin(2πt)), yaw = 0.5·sin(2πt)
OdomSample sample_at(double t)
{
    OdomSample s;
    s.t = SampleTime{t};
    const double w = 2.0 * pm::kPi;
    s.pose.p = {{std::sin(w * t), std::cos(w * t), 0.5 * std::sin(w * t)}};
    s.pose.q = pm::quat_from_yaw(0.5 * std::sin(w * t));
    return s;
}

void fill_sine(OdomBuffer & buf, double t0, double t1, double dt = 0.004)
{
    for (double t = t0; t <= t1 + 1e-9; t += dt) {
        ASSERT_TRUE(buf.push(sample_at(t)));
    }
}

}  // namespace

// ---- UT-01: 解析正弦轨迹 250 Hz 采样, 任意时刻查询误差 < 1 mm ----
TEST(OdomBufferInterp, UT01_SineTrajectoryUnder1mm)
{
    OdomBuffer buf(4.0, 0.05);
    fill_sine(buf, 0.0, 2.0);
    // 非采样点密集查询 (0.0771 与 4 ms 采样周期互质, 覆盖各种插值相位)
    for (double t = 0.1003; t < 1.9; t += 0.0771) {
        pm::Pose out;
        ASSERT_EQ(buf.query(SampleTime{t}, &out), OdomBuffer::QueryResult::OK) << "t=" << t;
        const OdomSample ref = sample_at(t);
        EXPECT_NEAR(out.p[0], ref.pose.p[0], 1e-3);
        EXPECT_NEAR(out.p[1], ref.pose.p[1], 1e-3);
        EXPECT_NEAR(out.p[2], ref.pose.p[2], 1e-3);
        EXPECT_NEAR(pm::yaw_from_quat(out.q), pm::yaw_from_quat(ref.pose.q), 1e-4);
    }
    EXPECT_EQ(buf.too_old_count(), 0u);
    EXPECT_EQ(buf.too_new_count(), 0u);
}

// ---- UT-02: 边界 —— 早于最旧 / 晚于最新超外推上限, 拒绝并计数 ----
TEST(OdomBufferBoundary, UT02_TooOldRejectedAndCounted)
{
    OdomBuffer buf(1.0, 0.05);    // 缓冲仅 1 s
    fill_sine(buf, 0.0, 2.0);     // 旧样本被时长剔除, 最旧 ≈ 1.0 s
    pm::Pose out;
    EXPECT_EQ(buf.query(SampleTime{0.5}, &out), OdomBuffer::QueryResult::TOO_OLD);
    EXPECT_EQ(buf.too_old_count(), 1u);
    EXPECT_EQ(buf.query(SampleTime{0.2}, &out), OdomBuffer::QueryResult::TOO_OLD);
    EXPECT_EQ(buf.too_old_count(), 2u);
}

TEST(OdomBufferBoundary, UT02_TooNewBeyondExtrapolationRejected)
{
    OdomBuffer buf(4.0, 0.05);
    fill_sine(buf, 0.0, 1.0);
    pm::Pose out;
    // 超外推上限 → TOO_NEW
    EXPECT_EQ(buf.query(SampleTime{1.0 + 0.051}, &out), OdomBuffer::QueryResult::TOO_NEW);
    EXPECT_EQ(buf.too_new_count(), 1u);
    // 外推上限之内 → 零阶保持返回最新样本
    ASSERT_EQ(buf.query(SampleTime{1.0 + 0.049}, &out), OdomBuffer::QueryResult::OK);
    const OdomSample newest = sample_at(1.0);
    EXPECT_NEAR(out.p[0], newest.pose.p[0], 1e-9);
    EXPECT_NEAR(out.p[1], newest.pose.p[1], 1e-9);
    EXPECT_EQ(buf.too_new_count(), 1u);    // 未再计数
}

// ---- UT-14 (缓冲部分): reset 清空 + 静默窗口丢弃旧系尾巴 ----
TEST(OdomBufferReset, UT14_ClearAndSettleDropsStaleSamples)
{
    OdomBuffer buf(4.0, 0.05);
    fill_sine(buf, 0.0, 1.0);
    ASSERT_GT(buf.size(), 0u);

    // reset 于 t=1.0, 静默窗口至 1.05
    buf.clear_and_settle(SampleTime{1.05});
    EXPECT_EQ(buf.size(), 0u);

    // 窗口内到达的旧坐标系样本 (乱序尾巴) 直接丢弃并计数
    EXPECT_FALSE(buf.push(sample_at(1.004)));
    EXPECT_FALSE(buf.push(sample_at(1.008)));
    EXPECT_EQ(buf.settle_drop_count(), 2u);
    EXPECT_EQ(buf.size(), 0u);

    // 窗口内查询: 缓冲为空 → EMPTY, 计入 too_old (详设 4.6 附带动作 3)
    pm::Pose out;
    EXPECT_EQ(buf.query(SampleTime{1.02}, &out), OdomBuffer::QueryResult::EMPTY);
    EXPECT_EQ(buf.too_old_count(), 1u);

    // 窗口后样本正常入缓冲, 跨 reset 无错误插值结果产出
    EXPECT_TRUE(buf.push(sample_at(1.06)));
    EXPECT_TRUE(buf.push(sample_at(1.064)));
    ASSERT_EQ(buf.query(SampleTime{1.062}, &out), OdomBuffer::QueryResult::OK);
    const OdomSample ref = sample_at(1.062);
    EXPECT_NEAR(out.p[0], ref.pose.p[0], 1e-3);
}

// ---- F04: 非有限样本在入口被拦截, 插值窗口免疫 ----
TEST(OdomBufferGuard, F04_NonFiniteSampleDropped)
{
    OdomBuffer buf(4.0, 0.05);
    fill_sine(buf, 0.0, 0.5);
    const std::size_t n = buf.size();

    OdomSample bad = sample_at(0.504);
    bad.pose.p[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(buf.push(bad));
    OdomSample bad_q = sample_at(0.508);
    bad_q.pose.q.w = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(buf.push(bad_q));
    EXPECT_EQ(buf.invalid_drop_count(), 2u);
    EXPECT_EQ(buf.size(), n);    // 缓冲未被污染

    // 后续正常样本与插值不受影响
    ASSERT_TRUE(buf.push(sample_at(0.512)));
    pm::Pose out;
    ASSERT_EQ(buf.query(SampleTime{0.506}, &out), OdomBuffer::QueryResult::OK);
    EXPECT_TRUE(std::isfinite(out.p[2]));
    const OdomSample ref = sample_at(0.506);
    EXPECT_NEAR(out.p[0], ref.pose.p[0], 1e-3);
}

// ---- 附加: 时间戳乱序/重复防御 ----
TEST(OdomBufferGuard, DisorderedTimestampDropped)
{
    OdomBuffer buf(4.0, 0.05);
    ASSERT_TRUE(buf.push(sample_at(1.0)));
    EXPECT_FALSE(buf.push(sample_at(0.9)));     // 回退
    EXPECT_FALSE(buf.push(sample_at(1.0)));     // 重复
    EXPECT_EQ(buf.disorder_drop_count(), 2u);
    EXPECT_EQ(buf.size(), 1u);
}

// ---- 附加: 缓冲时长剔除 (内存有界) ----
TEST(OdomBufferGuard, OldSamplesPrunedByDuration)
{
    OdomBuffer buf(1.0, 0.05);
    fill_sine(buf, 0.0, 3.0);
    EXPECT_GE(buf.oldest_time().s, buf.newest_time().s - 1.0 - 1e-9);
    // 250 Hz × 1 s ≈ 251 条
    EXPECT_LE(buf.size(), 260u);
}

// ---- 附加: 精确命中样本时刻 ----
TEST(OdomBufferInterp, ExactSampleTimeHit)
{
    OdomBuffer buf(4.0, 0.05);
    fill_sine(buf, 0.0, 1.0);
    pm::Pose out;
    ASSERT_EQ(buf.query(SampleTime{0.5}, &out), OdomBuffer::QueryResult::OK);
    const OdomSample ref = sample_at(0.5);
    EXPECT_NEAR(out.p[0], ref.pose.p[0], 1e-12);
    EXPECT_NEAR(out.p[1], ref.pose.p[1], 1e-12);
    EXPECT_NEAR(out.p[2], ref.pose.p[2], 1e-12);
}

// ---- 附加: 空缓冲查询 ----
TEST(OdomBufferBoundary, EmptyBufferQueryCountsAsTooOld)
{
    OdomBuffer buf(4.0, 0.05);
    pm::Pose out;
    EXPECT_EQ(buf.query(SampleTime{1.0}, &out), OdomBuffer::QueryResult::EMPTY);
    EXPECT_EQ(buf.too_old_count(), 1u);
}

// ---- newest_pose: ③修法评估点来源 (最新样本位姿访问器) ----
TEST(OdomBufferAccess, NewestPoseIsLastPushed)
{
    OdomBuffer buf(4.0, 0.05);
    fill_sine(buf, 0.0, 0.5);
    ASSERT_FALSE(buf.empty());
    const OdomSample last = sample_at(0.5);
    EXPECT_NEAR(buf.newest_pose().p[0], last.pose.p[0], 1e-12);
    EXPECT_NEAR(buf.newest_pose().p[1], last.pose.p[1], 1e-12);
    EXPECT_NEAR(buf.newest_pose().p[2], last.pose.p[2], 1e-12);
    EXPECT_NEAR(buf.newest_time().s, 0.5, 1e-12);
}

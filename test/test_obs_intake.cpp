/**
 * test_obs_intake.cpp
 * 修正源两票 processor 链测试 (设计文档 v1 五节 5.2):
 *   - FiniteGuard 生死票: NaN 帧废弃并计数, 正常帧放行
 *   - 链短路: usable=false 后后续 processor 不执行
 *   - 降权票: clean=false 时帧仍可用, quality 修改随帧透传
 *   - reset_all 传导到链上每个 processor
 */

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <memory>

#include "map_odom_bias/core/obs_intake.hpp"

using map_odom_bias::BiasEstimator;
using map_odom_bias::BiasEstimatorParams;
using map_odom_bias::FiniteGuard;
using map_odom_bias::GlobalPoseObservation;
using map_odom_bias::HostTime;
using map_odom_bias::ObsIntake;
using map_odom_bias::ObsProcessor;
using map_odom_bias::ProcResult;
using map_odom_bias::SampleTime;

namespace
{

GlobalPoseObservation make_obs(double x = 0.1)
{
    GlobalPoseObservation o;
    o.t_sample = SampleTime{1.0};
    o.t_arrival = HostTime{1.1};
    o.T_obs.x = x;
    o.p_ob = {{2.0, 0.0, 0.5}};
    return o;
}

/// 探针 processor: 记录调用与 reset, 可配置两票与 quality 动作
class Probe : public ObsProcessor
{
public:
    explicit Probe(ProcResult verdict, double quality_factor = 1.0)
    : verdict_(verdict), quality_factor_(quality_factor)
    {
    }

    ProcResult process(GlobalPoseObservation & obs,
                       const BiasEstimator &) override
    {
        ++calls;
        obs.quality *= quality_factor_;
        return verdict_;
    }

    void reset() override { ++resets; }

    int calls{0};
    int resets{0};

private:
    ProcResult verdict_;
    double quality_factor_;
};

}  // namespace

TEST(ObsIntake, FiniteGuardDropsNanKeepsNormal)
{
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    intake.add(std::unique_ptr<ObsProcessor>(new FiniteGuard));

    GlobalPoseObservation good = make_obs();
    EXPECT_TRUE(intake.run(good, host));
    EXPECT_EQ(intake.dropped_count(), 0u);

    GlobalPoseObservation bad = make_obs();
    bad.T_obs.yaw = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(intake.run(bad, host));
    EXPECT_EQ(intake.dropped_count(), 1u);

    GlobalPoseObservation bad_p = make_obs();
    bad_p.p_ob[1] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(intake.run(bad_p, host));
    EXPECT_EQ(intake.dropped_count(), 2u);
}

TEST(ObsIntake, ChainShortCircuitsOnUnusable)
{
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    auto * killer = new Probe(ProcResult{true, false});    // 生死票废弃
    auto * after = new Probe(ProcResult{true, true});
    intake.add(std::unique_ptr<ObsProcessor>(killer));
    intake.add(std::unique_ptr<ObsProcessor>(after));

    GlobalPoseObservation o = make_obs();
    EXPECT_FALSE(intake.run(o, host));
    EXPECT_EQ(killer->calls, 1);
    EXPECT_EQ(after->calls, 0);    // 短路: 不再执行
    EXPECT_EQ(intake.dropped_count(), 1u);
}

TEST(ObsIntake, DirtyFrameStillUsableWithDowngradedQuality)
{
    // 降权票: clean=false 帧仍可用, quality 修改透传 (v2 软降权的输入)
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    intake.add(std::unique_ptr<ObsProcessor>(
        new Probe(ProcResult{false, true}, 0.25)));

    GlobalPoseObservation o = make_obs();
    ASSERT_NEAR(o.quality, 1.0, 1e-12);
    EXPECT_TRUE(intake.run(o, host));
    EXPECT_NEAR(o.quality, 0.25, 1e-12);
    EXPECT_EQ(intake.dropped_count(), 0u);
}

TEST(ObsIntake, ResetAllReachesEveryProcessor)
{
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    auto * a = new Probe(ProcResult{true, true});
    auto * b = new Probe(ProcResult{true, true});
    intake.add(std::unique_ptr<ObsProcessor>(a));
    intake.add(std::unique_ptr<ObsProcessor>(b));
    intake.reset_all();
    EXPECT_EQ(a->resets, 1);
    EXPECT_EQ(b->resets, 1);
}

// ================= v2 质量链 processor (设计文档 v1 5.2) =================

using map_odom_bias::MedianWindow;
using map_odom_bias::TagCountQuality;
namespace pm2 = map_odom_bias::pose_math;

TEST(MedianWindow, FiltersSpikeKeepsSustainedChange)
{
    // 自估计器 F21 迁入 (行为保真, 位置改门控前): 单帧毛刺滤除,
    // 持续真实变化经 ~window/2 帧延迟透传
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    intake.add(std::unique_ptr<ObsProcessor>(new MedianWindow(3)));

    auto run_x = [&](double x) {
        GlobalPoseObservation o = make_obs(x);
        EXPECT_TRUE(intake.run(o, host));
        return o.T_obs.x;
    };
    EXPECT_NEAR(run_x(0.0), 0.0, 1e-12);
    EXPECT_NEAR(run_x(0.0), 0.0, 1e-12);
    EXPECT_NEAR(run_x(0.05), 0.0, 1e-12);    // {0,0,0.05} 中值 = 0 毛刺滤除
    EXPECT_NEAR(run_x(0.0), 0.0, 1e-12);
    EXPECT_NEAR(run_x(0.05), 0.05, 1e-12);   // {0.05,0,0.05} 持续变化进账
    EXPECT_NEAR(run_x(0.05), 0.05, 1e-12);
}

TEST(MedianWindow, YawWrapSafeNearPi)
{
    // yaw 以窗口首元素为基准取相对角中值, ±π 环绕点附近不塌到 0
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    intake.add(std::unique_ptr<ObsProcessor>(new MedianWindow(3)));

    auto run_yaw = [&](double yaw) {
        GlobalPoseObservation o = make_obs(0.0);
        o.T_obs.yaw = yaw;
        EXPECT_TRUE(intake.run(o, host));
        return o.T_obs.yaw;
    };
    run_yaw(3.1);
    run_yaw(-3.1);    // 与 3.1 相隔仅 0.083 rad (跨环绕点)
    const double m = run_yaw(3.1);
    EXPECT_NEAR(pm2::wrap_angle(m - 3.1), 0.0, 1e-9);    // 中值贴 3.1 侧
}

TEST(MedianWindow, ResetClearsWindowState)
{
    // reset_all 经 unique_ptr 虚调用真清到实例 (MRS 按值拷贝坑对治);
    // 参考系事件后旧坐标系历史不再污染中值
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    intake.add(std::unique_ptr<ObsProcessor>(new MedianWindow(3)));

    for (int i = 0; i < 3; ++i) {
        GlobalPoseObservation o = make_obs(5.0);    // 旧坐标系观测
        intake.run(o, host);
    }
    intake.reset_all();
    GlobalPoseObservation fresh = make_obs(1.0);    // 新坐标系首帧
    EXPECT_TRUE(intake.run(fresh, host));
    EXPECT_NEAR(fresh.T_obs.x, 1.0, 1e-12);         // 不被旧值 5.0 拖住
}

TEST(MedianWindow, WindowOnePassesThrough)
{
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    intake.add(std::unique_ptr<ObsProcessor>(new MedianWindow(1)));
    GlobalPoseObservation o = make_obs(0.37);
    EXPECT_TRUE(intake.run(o, host));
    EXPECT_NEAR(o.T_obs.x, 0.37, 1e-12);    // 逐位直通 (默认关闭零差异)
}

TEST(TagCountQuality, TiersByTagCount)
{
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    intake.add(std::unique_ptr<ObsProcessor>(new TagCountQuality(0.4, 0.7)));

    auto quality_for = [&](int tags) {
        GlobalPoseObservation o = make_obs();
        o.tag_count = tags;
        EXPECT_TRUE(intake.run(o, host));
        return o.quality;
    };
    EXPECT_NEAR(quality_for(-1), 1.0, 1e-12);    // 未提供 → 直通满质量
    EXPECT_NEAR(quality_for(5), 1.0, 1e-12);
    EXPECT_NEAR(quality_for(3), 1.0, 1e-12);
    EXPECT_NEAR(quality_for(2), 0.7, 1e-12);
    EXPECT_NEAR(quality_for(1), 0.4, 1e-12);
    EXPECT_NEAR(quality_for(0), 0.4, 1e-12);     // 0-tag 有 pose = 上游异常,
                                                  // 并入最低档保守处理
}

TEST(TagCountQuality, ComposesMultiplicativelyWithUpstream)
{
    // quality 乘法复合: 上游 processor 已降到 0.5, 单 tag 档 0.4 → 0.2
    BiasEstimator host{BiasEstimatorParams{}};
    ObsIntake intake;
    intake.add(std::unique_ptr<ObsProcessor>(
        new Probe(ProcResult{false, true}, 0.5)));
    intake.add(std::unique_ptr<ObsProcessor>(new TagCountQuality(0.4, 0.7)));

    GlobalPoseObservation o = make_obs();
    o.tag_count = 1;
    EXPECT_TRUE(intake.run(o, host));
    EXPECT_NEAR(o.quality, 0.2, 1e-12);
}

TEST(TagCountQuality, CtorValidatesTiers)
{
    // 加载期硬断言: 0 < single ≤ dual ≤ 1 (fail-fast 哲学同三带 Δ≤L)
    EXPECT_THROW(TagCountQuality(0.7, 0.4), std::invalid_argument);
    EXPECT_THROW(TagCountQuality(0.0, 0.7), std::invalid_argument);
    EXPECT_THROW(TagCountQuality(0.4, 1.5), std::invalid_argument);
    EXPECT_NO_THROW(TagCountQuality(0.4, 0.7));
    EXPECT_NO_THROW(TagCountQuality(1.0, 1.0));
}

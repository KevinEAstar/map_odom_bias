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

/**
 * test_time_types.cpp
 * 钟域 strong typedef 性质测试 (设计文档 v1 七节):
 *   - 同域 age / 跨域 delay 的数值语义
 *   - 类型区分: SampleTime 与 HostTime 不可隐式互换 (编译期约束,
 *     以 static_assert 锁定非可转换性)
 */

#include <gtest/gtest.h>

#include <type_traits>

#include "map_odom_bias/core/time_types.hpp"

using map_odom_bias::HostTime;
using map_odom_bias::SampleTime;

TEST(TimeTypes, SameDomainAge)
{
    EXPECT_NEAR(age_of(HostTime{10.0}, HostTime{12.5}), 2.5, 1e-12);
    EXPECT_NEAR(age_of(SampleTime{100.0}, SampleTime{100.1}), 0.1, 1e-12);
}

TEST(TimeTypes, CrossDomainOnlyViaExplicitFunctions)
{
    // 端到端延迟 = 到达刻 − 采样刻 (同源钟假设的显式出口)
    EXPECT_NEAR(map_odom_bias::delay_assuming_same_clock(
                    SampleTime{5.0}, HostTime{5.231}),
                0.231, 1e-12);
    const SampleTime st =
        map_odom_bias::as_sample_time_assuming_same_clock(HostTime{7.0});
    EXPECT_NEAR(st.s, 7.0, 1e-12);
}

TEST(TimeTypes, DomainsAreDistinctTypes)
{
    // 钟域不可隐式互换: 裸传错域编译不过 (类型化的核心目的)
    static_assert(!std::is_convertible<SampleTime, HostTime>::value,
                  "SampleTime 不得隐式转 HostTime");
    static_assert(!std::is_convertible<HostTime, SampleTime>::value,
                  "HostTime 不得隐式转 SampleTime");
    static_assert(!std::is_convertible<double, SampleTime>::value,
                  "裸 double 不得隐式转 SampleTime");
    SUCCEED();
}

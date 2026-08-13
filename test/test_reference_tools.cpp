/**
 * test_reference_tools.cpp
 * 消费者工具件测试 (设计文档 v1 六节 6.2, MRS A 报告 Q7 建议 1-3;
 * 含 08-12 对抗复核 H1-H3/M4-M10 修复的回归用例):
 *   - 编译期反例 (static_assert): Anchored 无叠加接口 / Rebasable 无
 *     resolve / 两类型不可互构 / delta 快捷路径仅 Point 特化 /
 *     OneShotAnchored 禁拷贝且左值不可取值 / 公开类全 final
 *   - Anchored 拒空 frame_id (MRS A' 穿透口对治) + move 落拷贝保不变量
 *   - rebase(pre, post) 配对 = 帧改写 delta 精确恢复, 跨号免疫, 单调守卫
 *   - apply_delta 相邻性强制 + acknowledge 显式确认 + resync 恢复通道
 *   - RebaseBus 幂等 / 跳号标记 / 全员分发 / 异常后事件可重投 / 重入
 *     拒绝 / 轮内注销即时生效
 *   - ResetTransaction 语义性恢复 + 带号校验
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "map_odom_bias/tools/reference_tools.hpp"

using map_odom_bias::pose_math::compose;
using map_odom_bias::pose_math::inverse;
using map_odom_bias::pose_math::kPi;
using map_odom_bias::pose_math::make_heading_reset_delta;
using map_odom_bias::pose_math::Pose;
using map_odom_bias::pose_math::quat_from_yaw;
using map_odom_bias::pose_math::Transform4D;
using map_odom_bias::pose_math::yaw_from_quat;
using map_odom_bias::tools::Anchored;
using map_odom_bias::tools::OneShotAnchored;
using map_odom_bias::tools::Point;
using map_odom_bias::tools::Rebasable;
using map_odom_bias::tools::RebaseBus;
using map_odom_bias::tools::ResetEventData;
using map_odom_bias::tools::ResetTransaction;

namespace
{

// ---- C++14 检测惯用法 (make_void 包装规避 CWG 1558) ----
template<class... Ts>
struct make_void
{
    using type = void;
};
template<class... Ts>
using void_t = typename make_void<Ts...>::type;

template<class T, class = void>
struct has_rebase : std::false_type {};
template<class T>
struct has_rebase<T, void_t<decltype(std::declval<T &>().rebase(
    std::declval<const Transform4D &>(), std::declval<const Transform4D &>(),
    std::uint32_t{}))>> : std::true_type {};

template<class T, class = void>
struct has_lvalue_resolve : std::false_type {};
template<class T>
struct has_lvalue_resolve<T,
    void_t<decltype(std::declval<T &>().resolve_with(
        std::declval<const Transform4D &>()))>> : std::true_type {};

template<class T, class = void>
struct has_rvalue_resolve : std::false_type {};
template<class T>
struct has_rvalue_resolve<T,
    void_t<decltype(std::declval<T &&>().resolve_with(
        std::declval<const Transform4D &>()))>> : std::true_type {};

template<class T, class = void>
struct has_apply_delta : std::false_type {};
template<class T>
struct has_apply_delta<T, void_t<decltype(std::declval<T &>().apply_delta(
    std::declval<const ResetEventData &>()))>> : std::true_type {};

template<class T, class = void>
struct has_acknowledge : std::false_type {};
template<class T>
struct has_acknowledge<T, void_t<decltype(std::declval<T &>().acknowledge(
    std::uint32_t{}))>> : std::true_type {};

// ---- 编译期反例: 互斥规则编成类型, 双重补偿写不出来 ----
static_assert(!has_rebase<Anchored<Point>>::value,
              "Anchored 不得有 rebase (经变换出口禁叠)");
static_assert(!has_apply_delta<Anchored<Point>>::value,
              "Anchored 不得有 apply_delta");
static_assert(!has_acknowledge<Anchored<Point>>::value,
              "Anchored 不得有 acknowledge");
static_assert(has_lvalue_resolve<Anchored<Point>>::value,
              "Anchored 必须有 resolve_with (唯一取值通道)");
static_assert(!has_lvalue_resolve<Rebasable<Point>>::value,
              "Rebasable 不得有 resolve_with (裸值不经变换解释)");
static_assert(has_rebase<Rebasable<Point>>::value,
              "Rebasable<Point> 必须有 rebase");
static_assert(has_rebase<Rebasable<Pose>>::value,
              "Rebasable<Pose> 必须有 rebase");
static_assert(has_apply_delta<Rebasable<Point>>::value,
              "delta 快捷路径限 Point 特化 (设计文档 v1 6.1)");
static_assert(!has_apply_delta<Rebasable<Pose>>::value,
              "结构化状态禁用 delta 快捷路径");
static_assert(!std::is_constructible<Anchored<Point>,
                                     const Rebasable<Point> &>::value,
              "Rebasable 不可隐式变 Anchored (升格只经 as_anchored)");
static_assert(!std::is_constructible<Rebasable<Point>,
                                     const Anchored<Point> &>::value,
              "Anchored 不可降格为 Rebasable");
static_assert(!std::is_convertible<Rebasable<Point>, Anchored<Point>>::value,
              "无隐式转换通道");
static_assert(!std::is_convertible<Anchored<Point>, Rebasable<Point>>::value,
              "无隐式转换通道");

// ---- 一次性视图: 取值即耗尽, 长持语法上写不出 (复核 H1 封堵) ----
static_assert(!has_lvalue_resolve<OneShotAnchored<Point>>::value,
              "OneShotAnchored 左值不可取值 (&&+禁拷贝 = 一次性)");
static_assert(has_rvalue_resolve<OneShotAnchored<Point>>::value,
              "OneShotAnchored 右值可取值");
static_assert(!std::is_copy_constructible<OneShotAnchored<Point>>::value,
              "OneShotAnchored 禁拷贝");
static_assert(!std::is_copy_assignable<OneShotAnchored<Point>>::value,
              "OneShotAnchored 禁拷贝赋值");

// ---- 公开类封死派生 (复核 M5: 派生复活被禁接口的通道关闭) ----
static_assert(std::is_final<Rebasable<Point>>::value, "Rebasable 应为 final");
static_assert(std::is_final<Rebasable<Pose>>::value, "Rebasable 应为 final");
static_assert(std::is_final<Anchored<Point>>::value, "Anchored 应为 final");
static_assert(std::is_final<OneShotAnchored<Point>>::value,
              "OneShotAnchored 应为 final");
static_assert(std::is_final<RebaseBus>::value, "RebaseBus 应为 final");
static_assert(std::is_final<ResetTransaction>::value,
              "ResetTransaction 应为 final");

const Point kAnchor = {{5.0, 1.0, 0.0}};

/// 限定包装: 裸 apply(t, array) 会经 ADL 撞上 std::apply (libstdc++ 的
/// <tuple> 实现非 SFINAE 友好, C++14 下硬报错)
Point tf_point(const Transform4D & t, const Point & p)
{
    return map_odom_bias::pose_math::apply(t, p);
}

void expect_point_near(const Point & a, const Point & b, double tol = 1e-12)
{
    EXPECT_NEAR(a[0], b[0], tol);
    EXPECT_NEAR(a[1], b[1], tol);
    EXPECT_NEAR(a[2], b[2], tol);
}

ResetEventData make_event(std::uint32_t iteration,
                          const Transform4D & delta = Transform4D{},
                          ResetEventData::Cause cause =
                              ResetEventData::Cause::kEkfReset)
{
    ResetEventData e;
    e.cause = cause;
    e.iteration = iteration;
    e.delta = delta;
    return e;
}

}  // namespace

// ==================== Anchored ====================

TEST(Anchored, RejectsEmptyFrame)
{
    // MRS A' 缺口: 空 frame_id 被解析成"每拍刷新的默认帧", A 类容器装 B 类
    // 语义 —— 构造期直接拒绝
    EXPECT_THROW(Anchored<Point>(kAnchor, ""), std::invalid_argument);
    EXPECT_THROW(Anchored<Pose>(Pose{}, ""), std::invalid_argument);
}

TEST(Anchored, MoveFallsBackToCopyKeepingInvariant)
{
    // 复核 M4 回归: 隐式 move 被抑制, std::move 落到拷贝 —— 源对象
    // frame_id 不被搬空, 构造期拒空帧的不变量保持
    Anchored<Point> a(kAnchor, "map");
    Anchored<Point> b(std::move(a));
    EXPECT_EQ(a.frame_id(), "map");
    EXPECT_EQ(b.frame_id(), "map");
    expect_point_near(a.resolve_with(Transform4D{}), kAnchor);
}

TEST(Anchored, ResolveWithAppliesTransform)
{
    Anchored<Point> a({{1.0, 0.0, 0.0}}, "map");
    EXPECT_EQ(a.frame_id(), "map");

    Transform4D t;
    t.x = 0.5;
    t.yaw = kPi / 2.0;
    // Rz(90°)·(1,0,0) + (0.5,0,0) = (0.5, 1, 0)
    expect_point_near(a.resolve_with(t), {{0.5, 1.0, 0.0}});
    // 恒等变换 = 原系取值
    expect_point_near(a.resolve_with(Transform4D{}), {{1.0, 0.0, 0.0}});
}

TEST(Anchored, ResolvePoseTransformsPositionAndOrientation)
{
    Pose v;
    v.p = {{2.0, 0.0, 1.0}};
    v.q = quat_from_yaw(0.3);
    Anchored<Pose> a(v, "map");

    Transform4D t;
    t.x = 1.0;
    t.yaw = kPi / 2.0;
    const Pose out = a.resolve_with(t);
    // 位置: Rz(90°)·(2,0,1) + (1,0,0) = (1, 2, 1)
    expect_point_near(out.p, {{1.0, 2.0, 1.0}});
    EXPECT_NEAR(yaw_from_quat(out.q), 0.3 + kPi / 2.0, 1e-12);
}

// ==================== Rebasable: rebase(pre, post) 配对 ====================

TEST(Rebasable, RebaseRecoversFrameRewriteDeltaExactly)
{
    // EKF 航向 reset: odom 系被 D 改写, 机体位姿 T_pre → T_post = D·T_pre。
    // 消费者自持旧位姿配对首帧新位姿, 差分 post∘pre⁻¹ 精确恢复 D
    // (07-31 错拍单拍尖的结构性根治: 不依赖事件携带的 delta)
    const Point p_body = {{2.0, -1.0, 0.5}};
    const Transform4D D = make_heading_reset_delta(0.3, p_body);

    Transform4D T_pre;
    T_pre.x = 1.0;
    T_pre.y = 2.0;
    T_pre.z = 0.3;
    T_pre.yaw = 0.7;
    const Transform4D T_post = compose(D, T_pre);

    Rebasable<Point> r(kAnchor, 7);
    EXPECT_TRUE(r.rebase(T_pre, T_post, 8));
    expect_point_near(r.value(), tf_point(D, kAnchor));
    EXPECT_EQ(r.reference_iteration(), 8u);

    // 航向 reset 不动机体位置坐标 (make_heading_reset_delta 语义)
    expect_point_near(tf_point(D, p_body), p_body);
}

TEST(Rebasable, RebaseIsGapImmune)
{
    // 连环 reset 丢中间事件: (最旧, 最新) 一次配对 = 全部增量的复合
    Transform4D D1;
    D1.x = 0.1;
    D1.y = -0.2;
    D1.yaw = 0.05;
    const Transform4D D2 =
        make_heading_reset_delta(-0.4, {{1.0, 1.0, 0.0}});

    Transform4D T0;
    T0.yaw = 0.2;
    const Transform4D T1 = compose(D1, T0);
    const Transform4D T2 = compose(D2, T1);

    Rebasable<Point> r(kAnchor, 3);
    EXPECT_TRUE(r.rebase(T0, T2, 6));    // 跨 3 个号一次到位
    expect_point_near(r.value(), tf_point(compose(D2, D1), kAnchor));
    EXPECT_EQ(r.reference_iteration(), 6u);
}

TEST(Rebasable, SequentialRebasesEqualEndpointRebase)
{
    // 逐事件 rebase 两次 == 端点一次配对 (累计精度)
    Transform4D D1;
    D1.x = 0.3;
    D1.yaw = 0.4;
    const Transform4D D2 = make_heading_reset_delta(0.25, {{0.5, -0.5, 0.2}});

    Transform4D T0;
    T0.x = -1.0;
    T0.yaw = -0.3;
    const Transform4D T1 = compose(D1, T0);
    const Transform4D T2 = compose(D2, T1);

    Rebasable<Point> step(kAnchor, 0);
    EXPECT_TRUE(step.rebase(T0, T1, 1));
    EXPECT_TRUE(step.rebase(T1, T2, 2));

    Rebasable<Point> endpoint(kAnchor, 0);
    EXPECT_TRUE(endpoint.rebase(T0, T2, 2));

    expect_point_near(step.value(), endpoint.value());
}

TEST(Rebasable, RebaseMonotonicGuardRefusesReplayAndRollback)
{
    // 复核 M10 回归: 重放 (同号) 与倒退号拒绝, 值原样
    Transform4D T_pre;
    Transform4D T_post;
    T_post.x = 1.0;

    Rebasable<Point> r(kAnchor, 5);
    EXPECT_FALSE(r.rebase(T_pre, T_post, 5));    // 同号 = 重放
    EXPECT_FALSE(r.rebase(T_pre, T_post, 2));    // 倒退
    expect_point_near(r.value(), kAnchor);
    EXPECT_EQ(r.reference_iteration(), 5u);
}

TEST(Rebasable, RebasePoseTransformsPositionAndYaw)
{
    Pose v;
    v.p = {{1.0, 0.0, 0.0}};
    v.q = quat_from_yaw(0.2);

    Transform4D T_pre;
    Transform4D T_post;
    T_post.x = 2.0;
    T_post.yaw = kPi / 2.0;

    Rebasable<Pose> r(v, 0);
    EXPECT_TRUE(r.rebase(T_pre, T_post, 1));
    // (post∘pre⁻¹) = post: 位置 Rz(90°)·(1,0,0)+(2,0,0) = (2,1,0)
    expect_point_near(r.value().p, {{2.0, 1.0, 0.0}});
    EXPECT_NEAR(yaw_from_quat(r.value().q), 0.2 + kPi / 2.0, 1e-12);
}

// ==================== Rebasable<Point>: delta 快捷路径 ====================

TEST(Rebasable, DeltaShortcutMatchesRebaseWhenContiguous)
{
    // 同一事件走两条路径, 结果必须一致 (delta = post∘pre⁻¹ 时)
    const Transform4D D = make_heading_reset_delta(0.3, {{2.0, -1.0, 0.5}});
    Transform4D T_pre;
    T_pre.x = 0.4;
    T_pre.yaw = -0.6;
    const Transform4D T_post = compose(D, T_pre);

    Rebasable<Point> via_delta(kAnchor, 7);
    EXPECT_TRUE(via_delta.apply_delta(make_event(8, D)));
    EXPECT_EQ(via_delta.reference_iteration(), 8u);

    Rebasable<Point> via_rebase(kAnchor, 7);
    EXPECT_TRUE(via_rebase.rebase(T_pre, T_post, 8));

    expect_point_near(via_delta.value(), via_rebase.value());
    expect_point_near(via_delta.value(), tf_point(D, kAnchor));
}

TEST(Rebasable, DeltaShortcutRefusesGapAndReplay)
{
    Rebasable<Point> r(kAnchor, 7);

    // 跳号: 丢失的中间 delta 无法补 → 拒绝, 值与计数原样
    EXPECT_FALSE(r.apply_delta(make_event(10)));
    expect_point_near(r.value(), kAnchor);
    EXPECT_EQ(r.reference_iteration(), 7u);

    // 重放旧事件同样拒绝 (幂等)
    EXPECT_FALSE(r.apply_delta(make_event(7)));
    expect_point_near(r.value(), kAnchor);
}

TEST(Rebasable, AcknowledgeAdvancesPastIrrelevantEvent)
{
    // 混合 cause 场景: odom 系锚点对 SOURCE_JUMP 事件显式确认 (值不动,
    // 计数推进), 保住后续 EKF_RESET delta 的相邻性判定
    const Transform4D D = make_heading_reset_delta(0.2, {{0.0, 0.0, 0.0}});

    Rebasable<Point> r(kAnchor, 4);
    EXPECT_TRUE(r.acknowledge(5));      // SOURCE_JUMP @5 与本量无关
    EXPECT_EQ(r.reference_iteration(), 5u);
    expect_point_near(r.value(), kAnchor);

    EXPECT_TRUE(r.apply_delta(make_event(6, D)));    // @6 相邻可用
    expect_point_near(r.value(), tf_point(D, kAnchor));
    EXPECT_EQ(r.reference_iteration(), 6u);

    EXPECT_FALSE(r.acknowledge(9));     // 跳号确认同样拒绝
    EXPECT_EQ(r.reference_iteration(), 6u);
}

TEST(Rebasable, ResyncRecoversFromGap)
{
    // 复核 M9 回归: 跳号掉队后 delta/acknowledge 全拒 —— resync 以新参考
    // 系下重测的值 + 当前号重定基线, 之后相邻事件恢复可用
    const Transform4D D = make_heading_reset_delta(0.1, {{1.0, 0.0, 0.0}});

    Rebasable<Point> r(kAnchor, 2);
    EXPECT_FALSE(r.apply_delta(make_event(5)));      // 掉队
    EXPECT_FALSE(r.acknowledge(5));

    const Point remeasured = {{4.0, 2.0, 0.1}};
    r.resync(remeasured, 5);                          // 显式重建
    expect_point_near(r.value(), remeasured);
    EXPECT_EQ(r.reference_iteration(), 5u);

    EXPECT_TRUE(r.apply_delta(make_event(6, D)));    // 恢复相邻消费
    expect_point_near(r.value(), tf_point(D, remeasured));

    // 估计器重启号归零: resync 不做单调约束 (唯一允许倒退的通道)
    r.resync(remeasured, 0);
    EXPECT_EQ(r.reference_iteration(), 0u);
}

// ==================== 升格通道 (一次性视图) ====================

TEST(Rebasable, AsAnchoredYieldsOneShotView)
{
    // 复核 H1 封堵: 升格产物取值即耗尽 —— 链式现用现取是唯一形态,
    // "升格后长持跨 reset" (漏补偿镜像故障) 语法上写不出
    Rebasable<Point> r(kAnchor, 2);

    Transform4D t;
    t.x = 1.0;
    t.yaw = kPi;
    expect_point_near(r.as_anchored("odom").resolve_with(t),
                      tf_point(t, kAnchor), 1e-9);

    // 显式 move 后仍只能取一次 (frame_id 可查)
    OneShotAnchored<Point> view = r.as_anchored("odom");
    EXPECT_EQ(view.frame_id(), "odom");
    expect_point_near(std::move(view).resolve_with(Transform4D{}), kAnchor);

    // 升格同样拒空帧
    EXPECT_THROW(r.as_anchored(""), std::invalid_argument);
}

// ==================== RebaseBus ====================

TEST(RebaseBus, DispatchReachesAllRegistrantsIdempotently)
{
    RebaseBus bus;    // 基线 iteration=0
    int calls_a = 0;
    int calls_b = 0;
    bool last_contiguous = false;
    const int id_a = bus.register_handler(
        [&](const ResetEventData &, bool contiguous) {
            ++calls_a;
            last_contiguous = contiguous;
        });
    bus.register_handler(
        [&](const ResetEventData &, bool) { ++calls_b; });
    EXPECT_EQ(bus.size(), 2u);

    EXPECT_TRUE(bus.dispatch(make_event(1)));    // 全员分发 (漏注册对治)
    EXPECT_EQ(calls_a, 1);
    EXPECT_EQ(calls_b, 1);
    EXPECT_TRUE(last_contiguous);

    EXPECT_FALSE(bus.dispatch(make_event(1)));   // 重复事件幂等忽略
    EXPECT_EQ(calls_a, 1);

    EXPECT_FALSE(bus.dispatch(make_event(0)));   // 回退同样忽略

    EXPECT_TRUE(bus.dispatch(make_event(4)));    // 跳号: 分发但打非相邻标记
    EXPECT_FALSE(last_contiguous);
    EXPECT_EQ(bus.last_iteration(), 4u);

    bus.unregister(id_a);
    bus.unregister(9999);                        // 非法 id 无害
    EXPECT_TRUE(bus.dispatch(make_event(5)));
    EXPECT_EQ(calls_a, 2);                       // 已注销不再收
    EXPECT_EQ(calls_b, 3);
    EXPECT_EQ(bus.size(), 1u);
}

TEST(RebaseBus, SeedFromStatusIterationSetsBaseline)
{
    // 上电以 status.iteration 播种 (iteration 与 status 同源双载的用途):
    // 首个事件即可判相邻性; 构造参数与 seed 等价
    RebaseBus bus(40);
    bus.seed(41);
    bool contiguous = false;
    bus.register_handler(
        [&](const ResetEventData &, bool c) { contiguous = c; });

    EXPECT_TRUE(bus.dispatch(make_event(42)));
    EXPECT_TRUE(contiguous);
}

TEST(RebaseBus, HandlerThrowKeepsEventRedeliverable)
{
    // 复核 H3 回归: 号推进在全部 handler 之后 —— 中途抛异常则事件未
    // 消费, 重投可补; 已执行过的消费者由自身 iteration 守卫吸收重放
    RebaseBus bus;
    Rebasable<Point> anchor(kAnchor, 0);
    const Transform4D D = make_heading_reset_delta(0.2, {{1.0, 0.0, 0.0}});

    bus.register_handler(
        [&](const ResetEventData & e, bool) {
            anchor.apply_delta(e);    // 重投时守卫拒重放 (自幂等)
        });
    bool armed = true;
    bus.register_handler(
        [&](const ResetEventData &, bool) {
            if (armed) {
                throw std::runtime_error("handler 故障一次");
            }
        });

    EXPECT_THROW(bus.dispatch(make_event(1, D)), std::runtime_error);
    EXPECT_EQ(bus.last_iteration(), 0u);         // 号未推进

    armed = false;
    EXPECT_TRUE(bus.dispatch(make_event(1, D)));    // 重投成功
    EXPECT_EQ(bus.last_iteration(), 1u);
    expect_point_near(anchor.value(), tf_point(D, kAnchor));    // 只补偿一次
    EXPECT_EQ(anchor.reference_iteration(), 1u);
}

TEST(RebaseBus, ReentrantDispatchRefused)
{
    // 复核 M6 回归: handler 内再 dispatch 直接拒绝 (乱序投递会破坏
    // 相邻性判定)
    RebaseBus bus;
    bool inner_result = true;
    int calls = 0;
    bus.register_handler(
        [&](const ResetEventData &, bool) {
            ++calls;
            inner_result = bus.dispatch(make_event(2));
        });

    EXPECT_TRUE(bus.dispatch(make_event(1)));
    EXPECT_FALSE(inner_result);      // 重入被拒
    EXPECT_EQ(calls, 1);             // 事件 2 未投递
    EXPECT_EQ(bus.last_iteration(), 1u);

    EXPECT_TRUE(bus.dispatch(make_event(2)));    // 轮外正常投递
    EXPECT_EQ(calls, 2);
}

TEST(RebaseBus, InRoundUnregisterImmediateRegisterNextRound)
{
    // 复核 M7 回归: 轮内注销即时生效 (被注销者本轮剩余调用跳过,
    // 注销后销毁捕获对象安全); 轮内注册下轮才收到
    RebaseBus bus;
    int calls_b = 0;
    int calls_c = 0;
    int id_b = -1;

    bus.register_handler(
        [&](const ResetEventData &, bool) {
            bus.unregister(id_b);                       // 注销后位 handler
            bus.register_handler(                       // 轮内新注册
                [&](const ResetEventData &, bool) { ++calls_c; });
        });
    id_b = bus.register_handler(
        [&](const ResetEventData &, bool) { ++calls_b; });

    EXPECT_TRUE(bus.dispatch(make_event(1)));
    EXPECT_EQ(calls_b, 0);    // 本轮即被跳过
    EXPECT_EQ(calls_c, 0);    // 新注册者本轮不收

    EXPECT_TRUE(bus.dispatch(make_event(2)));
    EXPECT_EQ(calls_b, 0);
    EXPECT_EQ(calls_c, 1);    // 下轮生效
}

TEST(RebaseBus, HandlerCauseFilterPattern)
{
    // 收编样板: odom 系锚点只认 EKF_RESET, 其余 cause acknowledge 推号;
    // 跳号 (contiguous=false) 走 resync 重建。cause 过滤责任在 handler
    // 侧 (工具不硬编码"哪类量挂哪类事件")
    const Transform4D D = make_heading_reset_delta(0.3, {{1.0, 0.0, 0.0}});

    Rebasable<Point> odom_anchor(kAnchor, 0);
    const Point remeasured = {{2.0, 2.0, 0.0}};
    RebaseBus bus;
    bus.register_handler(
        [&](const ResetEventData & e, bool contiguous) {
            if (!contiguous) {
                // 实配中 = 新参考系下重测/经变换出口重算后 resync
                odom_anchor.resync(remeasured, e.iteration);
                return;
            }
            if (e.cause == ResetEventData::Cause::kEkfReset) {
                EXPECT_TRUE(odom_anchor.apply_delta(e));
            } else {
                EXPECT_TRUE(odom_anchor.acknowledge(e.iteration));
            }
        });

    bus.dispatch(make_event(1, Transform4D{},
                            ResetEventData::Cause::kSourceJump));
    expect_point_near(odom_anchor.value(), kAnchor);    // 值不动
    EXPECT_EQ(odom_anchor.reference_iteration(), 1u);   // 号已推进

    bus.dispatch(make_event(2, D));
    expect_point_near(odom_anchor.value(), tf_point(D, kAnchor));
    EXPECT_EQ(odom_anchor.reference_iteration(), 2u);

    bus.dispatch(make_event(5, D));                     // 跳号 → resync
    expect_point_near(odom_anchor.value(), remeasured);
    EXPECT_EQ(odom_anchor.reference_iteration(), 5u);
}

// ==================== ResetTransaction ====================

TEST(ResetTransaction, SettlesSemanticallyWithIterationCheck)
{
    // 复核 M8 回归: settle 带号校验 —— 旧代输出迟到不得误关新代窗口
    ResetTransaction tx;
    EXPECT_FALSE(tx.active());
    EXPECT_FALSE(tx.settle(0));         // 未开窗 settle 无效

    tx.open(5);                         // 事件到达: 下游判据关一拍
    EXPECT_TRUE(tx.active());
    EXPECT_EQ(tx.iteration(), 5u);

    tx.open(6);                         // 连环 reset: 保持关判据, 号覆盖
    EXPECT_TRUE(tx.active());

    EXPECT_FALSE(tx.settle(5));         // 旧代 (第 5 代) 首帧迟到 → 不关窗
    EXPECT_TRUE(tx.active());

    EXPECT_TRUE(tx.settle(6));          // 新参考系首帧输出 → 语义性恢复
    EXPECT_FALSE(tx.active());
    EXPECT_EQ(tx.iteration(), 6u);
}

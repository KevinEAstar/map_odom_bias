/**
 * @file reference_tools.hpp
 * @brief 消费者工具件: 参考系载体类型二分 + 重锚定总线 (设计文档 v1 六节
 *        6.2, MRS A 报告 Q7 建议 1-3 的类型化落地), header-only 零 ROS 依赖
 *
 * 互斥规则"经变换出口禁叠 / 不经变换必须叠"编成类型:
 *   - Anchored<T>:  长持有载体, **仅限帧定义稳定的量** (map 航点这类)。
 *     唯一取值通道 = resolve_with(变换), 无任何叠加/补偿接口 —— 消费者
 *     每次使用现取变换, 缓存后叠 delta 即双重补偿, 类型上写不出来。
 *     会被参考系事件改写的帧 (odom) 长持必须走 Rebasable: 本类型无补偿
 *     通道, odom 值跨 EKF reset 持有 = 值不补/变换补了, 两者叠加成漏账。
 *     构造拒空 frame_id (MRS 空帧解析成"每拍默认帧"是 A/B 二分穿透口);
 *     显式声明拷贝抑制隐式 move (搬空 frame_id 击穿构造期不变量)。
 *   - OneShotAnchored<T>: 裸值升格的一次性视图 (as_anchored 返回值):
 *     resolve_with 仅右值可调 (&&) 且禁拷贝 —— 取值即耗尽, "升格后长持
 *     跨 reset"语法上写不出; 需要长持的量本来就该是 Rebasable。
 *   - Rebasable<T>: 裸值 + reference_iteration, 参考系事件后必须补偿:
 *       rebase(pre_event_sample, post_event_sample, iter) —— 消费者自持
 *       旧值配对首帧新值, 差分 post∘pre⁻¹ 消除 delta 错拍类 bug (07-31
 *       单拍尖根治), 端点差分天然跨号免疫; 单调守卫拒绝重放/倒退;
 *       apply_delta(event) —— 快捷路径, 仅 Point 特化提供, 且强制事件号
 *       相邻 (丢帧即拒, 结构化状态禁用 delta, 见 msg/ResetEvent.msg);
 *       acknowledge(iter) —— 事件与本量无关时显式确认推号 (混合 cause 下
 *       维持相邻性判定, cause 过滤责任在总线 handler 侧);
 *       resync(value, iter) —— 跳号掉队 / 估计器重启号归零后的唯一恢复
 *       通道: 新参考系下重测(或经变换出口重算)的值显式重定基线;
 *       as_anchored(frame) 是裸值升格的唯一官方通道 (换帧复用唯一一条
 *       变换实现, 杜绝各处手写旋转)。
 *   - RebaseBus:    重锚定总线 —— 所有锚定量注册, ResetEvent 到达统一分发
 *     (08-02 align 锚点漏补 = 漏注册的直接对治); 重复/回退事件幂等忽略,
 *     跳号打 contiguous=false (delta 类路径必须拒绝, rebase 差分不受影响);
 *     上电以 status.iteration 播种基线 (iteration 同源双载的用途)。
 *     总线只做尽力去重 (seed 重播种 / 异常中断重投都可能造成重复投递),
 *     权威幂等在消费者侧 iteration 守卫 —— 这是分层约定, 不是缺陷。
 *   - ResetTransaction: 跳变窗口判据关一拍, 恢复条件 = 语义性 (下游在新
 *     参考系产出首帧输出时 settle, 带号校验防旧代 settle 关新代窗口),
 *     非定时。
 *
 * iteration 为 uint32 单调计数, 无回绕约定 (回绕需 40 亿次参考系事件,
 * 物理不可达; 不做回绕处理)。
 */

#ifndef MAP_ODOM_BIAS__TOOLS__REFERENCE_TOOLS_HPP_
#define MAP_ODOM_BIAS__TOOLS__REFERENCE_TOOLS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "map_odom_bias/core/pose_math.hpp"

namespace map_odom_bias
{
namespace tools
{

using Point = std::array<double, 3>;

/// ResetEvent.msg 的零 ROS 镜像 (ROS 侧订阅回调换算后进本结构)。
/// ⚠ delta 的坐标系语义随 cause 切换 (SOURCE_JUMP=map 系 / EKF_RESET=
/// odom 系), 工具不辨"这个事件是不是我的" —— cause 过滤必须由消费者在
/// 总线 handler 侧声明 (哪类量挂哪类事件)
struct ResetEventData
{
    enum class Cause : std::uint8_t
    {
        kSourceJump = 0,    // 观测跳变确认 (map 系点重定基增量)
        kEkfReset = 1,      // EKF reset 补偿 (odom 系改写左乘增量)
        kManual = 2,        // 人工干预 (预留)
    };

    Cause cause{Cause::kSourceJump};
    std::uint32_t iteration{0};
    pose_math::Transform4D delta;    // 便利品: 仅点类快捷路径消费
};

template<class T>
class OneShotAnchored;

namespace detail
{

inline Point apply_value(const pose_math::Transform4D & t, const Point & v)
{
    return pose_math::apply(t, v);
}

inline pose_math::Pose apply_value(const pose_math::Transform4D & t,
                                   const pose_math::Pose & v)
{
    return pose_math::apply_to_pose(t, v);
}

/// Rebasable 公共实现 (Point 特化在此之上加 delta 快捷路径)。
/// detail 内部件, 不作为公开扩展点; 数据 private, 派生只经 commit()
template<class T>
class RebasableBase
{
public:
    RebasableBase(const T & value, std::uint32_t reference_iteration)
    : value_(value), reference_iteration_(reference_iteration)
    {
    }

    const T & value() const { return value_; }
    std::uint32_t reference_iteration() const { return reference_iteration_; }

    /**
     * @brief (pre, post) 配对重定基: value ← (post∘pre⁻¹) ⊙ value
     *
     * ⚠ 入参是**同一物理量在事件前/后的两次表达** (典型 = 机体位姿快照:
     * 事件前最后一帧配事件后首帧), **绝不是 map→odom 出口变换** ——
     * 误传 t_map_odom_ctrl/cmd 会得到共轭错值 (纯平移 reset 下方向相反)。
     * 纯帧改写 (post = D∘pre) 时精确恢复 D; 端点差分对丢中间事件免疫,
     * new_iteration 任意跨号有效。
     *
     * @return 单调守卫: new_iteration ≤ 当前号 (重放/倒退) 拒绝返回 false;
     *         估计器重启号归零的场景走 resync, 不走本函数
     */
    bool rebase(const pose_math::Transform4D & pre_event_sample,
                const pose_math::Transform4D & post_event_sample,
                std::uint32_t new_iteration)
    {
        if (new_iteration <= reference_iteration_) {
            return false;
        }
        value_ = apply_value(
            pose_math::compose(post_event_sample,
                               pose_math::inverse(pre_event_sample)),
            value_);
        reference_iteration_ = new_iteration;
        return true;
    }

    /// 事件与本量无关时显式确认 (值不动, 推号); 仅相邻事件可确认 ——
    /// 跳号意味着有事件未审视, 不能宣称"与我无关", 应走 resync
    bool acknowledge(std::uint32_t event_iteration)
    {
        if (event_iteration != reference_iteration_ + 1) {
            return false;
        }
        reference_iteration_ = event_iteration;
        return true;
    }

    /// 显式重定基线: 新参考系下重测 (或经变换出口重算) 的值 + 当前号。
    /// 跳号掉队与估计器重启号归零的唯一恢复通道; 语义是重建不是补偿,
    /// 故不做单调约束
    void resync(const T & value, std::uint32_t iteration)
    {
        value_ = value;
        reference_iteration_ = iteration;
    }

    /// 裸值升格的唯一官方通道: 一次性视图, 取值即耗尽 (空 frame 拒绝)
    OneShotAnchored<T> as_anchored(const std::string & frame_id) const;

protected:
    /// 供 Point 特化的 delta 快捷路径提交 (数据不直接暴露给派生类)
    void commit(const T & value, std::uint32_t iteration)
    {
        value_ = value;
        reference_iteration_ = iteration;
    }

private:
    T value_;
    std::uint32_t reference_iteration_;
};

}  // namespace detail

/// 裸值载体: 必须随参考系事件补偿, 无 resolve (不经变换解释)
template<class T>
class Rebasable final : public detail::RebasableBase<T>
{
public:
    using detail::RebasableBase<T>::RebasableBase;
};

/// Point 特化: 独有 delta 快捷路径 (设计文档 v1 6.1 —— delta 仅供点类,
/// 结构化状态必须走 rebase(pre, post))
template<>
class Rebasable<Point> final : public detail::RebasableBase<Point>
{
public:
    using detail::RebasableBase<Point>::RebasableBase;

    /// 快捷路径: 仅相邻事件 (iteration+1) 接受; 跳号 = 中间 delta 已丢,
    /// 不可补 → 拒绝并保持原值 (重放旧事件同样拒绝, 幂等)
    bool apply_delta(const ResetEventData & e)
    {
        if (e.iteration != reference_iteration() + 1) {
            return false;
        }
        commit(pose_math::apply(e.delta, value()), e.iteration);
        return true;
    }
};

/// 长持有载体: 仅限帧定义稳定的量, 唯一取值通道 = 经变换出口, 无叠加
/// 接口 (双重补偿编译期写不出)。不携带 iteration —— 帧定义稳定的量与
/// 参考系事件计数无关, 携带一个从不校验的号只会诱导误用
template<class T>
class Anchored final
{
public:
    Anchored(const T & value, const std::string & frame_id)
    : value_(value), frame_id_(frame_id)
    {
        if (frame_id_.empty()) {
            throw std::invalid_argument(
                "Anchored: 空 frame_id 禁止 (A/B 二分穿透口, MRS A' 缺口)");
        }
    }

    // 显式声明拷贝 → 抑制隐式 move: string 被搬空后 frame_id 变空串,
    // 构造期拒空帧的不变量会被击穿; std::move 落到拷贝, 语义不变
    Anchored(const Anchored &) = default;
    Anchored & operator=(const Anchored &) = default;

    const std::string & frame_id() const { return frame_id_; }

    /// 唯一取值通道: t_target_from_frame 每次使用现取 (来自变换出口),
    /// 参考系事件由出口吸收, 本类型不做也不允许任何补偿叠加
    T resolve_with(const pose_math::Transform4D & t_target_from_frame) const
    {
        return detail::apply_value(t_target_from_frame, value_);
    }

private:
    T value_;
    std::string frame_id_;
};

/// 一次性升格视图: resolve_with 仅右值可调且禁拷贝 —— 取值即耗尽,
/// "升格后长持跨 reset"语法上写不出 (漏补偿镜像故障的类型级封堵)。
/// 需要长持的量本来就该留在 Rebasable 里随事件补偿
template<class T>
class OneShotAnchored final
{
public:
    OneShotAnchored(const T & value, const std::string & frame_id)
    : value_(value), frame_id_(frame_id)
    {
        if (frame_id_.empty()) {
            throw std::invalid_argument(
                "OneShotAnchored: 空 frame_id 禁止");
        }
    }

    OneShotAnchored(OneShotAnchored &&) = default;    // 仅为按值返回保留
    OneShotAnchored(const OneShotAnchored &) = delete;
    OneShotAnchored & operator=(const OneShotAnchored &) = delete;
    OneShotAnchored & operator=(OneShotAnchored &&) = delete;

    const std::string & frame_id() const { return frame_id_; }

    /// && 限定: 只能对临时 (或显式 std::move 后的) 对象取值一次
    T resolve_with(const pose_math::Transform4D & t_target_from_frame) &&
    {
        return detail::apply_value(t_target_from_frame, value_);
    }

private:
    T value_;
    std::string frame_id_;
};

namespace detail
{

template<class T>
OneShotAnchored<T> RebasableBase<T>::as_anchored(
    const std::string & frame_id) const
{
    return OneShotAnchored<T>(value_, frame_id);
}

}  // namespace detail

/// 重锚定总线: 锚定量统一注册, ResetEvent 统一分发 (漏注册对治)
class RebaseBus final
{
public:
    /// contiguous=false: 与上一已见事件跳号 —— delta 类快捷路径必须拒绝
    /// (走 resync 重建), rebase(pre, post) 差分路径不受影响。
    /// cause 过滤在 handler 侧 (哪类量挂哪类事件由消费者声明)
    using Handler = std::function<void(const ResetEventData & event,
                                       bool contiguous)>;

    /// @param initial_iteration 基线; 上电应以 status.iteration 播种
    explicit RebaseBus(std::uint32_t initial_iteration = 0)
    : last_iteration_(initial_iteration)
    {
    }

    /// 显式重播种 (估计器重启号归零的对接通道)。倒退播种会使已见事件
    /// 可再次分发 —— 去重是尽力而为, 权威幂等在消费者侧 iteration 守卫
    void seed(std::uint32_t iteration) { last_iteration_ = iteration; }

    /// @return 注销用句柄。dispatch 轮内注册: 下轮才收到分发
    int register_handler(Handler h)
    {
        const int id = next_id_++;
        handlers_.emplace_back(id, std::move(h));
        return id;
    }

    /// 即时生效: dispatch 轮内注销的 handler 本轮剩余调用同样跳过
    /// (注销后销毁 handler 捕获的对象是安全的)
    void unregister(int id)
    {
        for (std::size_t i = 0; i < handlers_.size(); ++i) {
            if (handlers_[i].first == id) {
                handlers_.erase(handlers_.begin() +
                                static_cast<std::ptrdiff_t>(i));
                return;
            }
        }
    }

    /**
     * @brief 事件分发
     * @return 是否分发 (重复/回退 iteration 幂等忽略, handler 内重入
     *         dispatch 直接拒绝 —— 均返回 false)
     *
     * 号推进在全部 handler 执行之后: 中途抛异常则事件未消费, 重投可补
     * (已执行过的消费者由其自身 iteration 守卫吸收重放)
     */
    bool dispatch(const ResetEventData & e)
    {
        if (dispatching_) {
            return false;    // 重入: 乱序投递会破坏相邻性判定
        }
        if (e.iteration <= last_iteration_) {
            return false;
        }
        const bool contiguous = (e.iteration == last_iteration_ + 1);

        // 快照 id 列表 + 逐 id 回查活表: 轮内注册下轮生效, 轮内注销
        // 即时生效; 调用前拷出 std::function, handler 自注销/注册引发
        // 的容器变动不碰执行中的函数对象
        std::vector<int> ids;
        ids.reserve(handlers_.size());
        for (const Entry & h : handlers_) {
            ids.push_back(h.first);
        }

        dispatching_ = true;
        struct DispatchGuard
        {
            bool & flag;
            ~DispatchGuard() { flag = false; }
        } guard{dispatching_};

        for (const int id : ids) {
            Handler fn;
            for (const Entry & h : handlers_) {
                if (h.first == id) {
                    fn = h.second;
                    break;
                }
            }
            if (fn) {
                fn(e, contiguous);
            }
        }
        last_iteration_ = e.iteration;
        return true;
    }

    std::uint32_t last_iteration() const { return last_iteration_; }
    std::size_t size() const { return handlers_.size(); }

private:
    using Entry = std::pair<int, Handler>;
    std::vector<Entry> handlers_;
    int next_id_{0};
    std::uint32_t last_iteration_;
    bool dispatching_{false};
};

/// reset 事务: 跳变窗口内下游判据 (divergence 消费/分歧门/控制误差监视)
/// 关一拍; 恢复 = 语义性 (新参考系下产出首帧输出时 settle), 非定时
class ResetTransaction final
{
public:
    void open(std::uint32_t iteration)
    {
        active_ = true;
        iteration_ = iteration;    // 连环 reset: 保持关判据, 号覆盖
    }

    /// @return 带号校验: 号不匹配 (旧代输出迟到) 或未开窗 → false 不关窗,
    ///         防旧代 settle 误关新代窗口
    bool settle(std::uint32_t iteration)
    {
        if (!active_ || iteration != iteration_) {
            return false;
        }
        active_ = false;
        return true;
    }

    bool active() const { return active_; }
    std::uint32_t iteration() const { return iteration_; }

private:
    bool active_{false};
    std::uint32_t iteration_{0};
};

}  // namespace tools
}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__TOOLS__REFERENCE_TOOLS_HPP_

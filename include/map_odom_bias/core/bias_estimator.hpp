/**
 * @file bias_estimator.hpp
 * @brief map→odom 偏置估计器: raw/ctrl 双状态 + 门控 + 慢吸收 + 状态机
 *        + EKF reset 瞬跳补偿 (详设 4.3~4.7 节), 纯逻辑无 ROS 依赖
 *
 * 三状态语义 (架构文档 3.3; cmd 为 B 方案追加, 2026-08):
 *   - raw  = 实测偏差账本, 采纳观测原值不做慢吸收 —— 监控要真;
 *            门控只负责把"不真的观测" (误检/双解翻转) 挡在账本之外
 *   - ctrl = 供控制"感知侧"消费的变换 (入口: 位置→map 垂足/进度/距离门),
 *            以一阶低通 + 速率 clamp 向 raw 慢速收敛 —— 感知要稳
 *            (垂足段号只前移不回头, 毛刺跳段不可逆)
 *   - cmd  = 供控制"指令侧"消费的变换 (出口: map 系 sp → odom 系发布),
 *            同一吸收律但独立快 τ_f 与独立钳位 —— 指令要贴 raw,
 *            关闭"稳态滞后 = 漂移率×τ"的出口欠账 (B 方案出口快通道)。
 *            cmd 不进感知反馈环, 无 ctrl 慢吸收那条正反馈路径, 钳位可放宽
 *
 * 时间以参数传入 (double 秒, 本地 ROS 钟), 不内部取时钟 —— 合成观测序列
 * 单元测试的结构前提 (详设第五节)。
 *
 * EKF reset (详设 4.6): reset 是坐标系事件而非观测误差, 门控与慢吸收都
 * 不适用; apply_reset(D) 接收已转 ENU 的 odom 系改写左乘增量 D, 对
 * raw/ctrl 同步施加 ·D⁻¹, 保证机体 map 系位姿跨 reset 连续。
 */

#ifndef MAP_ODOM_BIAS__CORE__BIAS_ESTIMATOR_HPP_
#define MAP_ODOM_BIAS__CORE__BIAS_ESTIMATOR_HPP_

#include <cstdint>
#include <deque>
#include <vector>

#include "map_odom_bias/core/pose_math.hpp"

namespace map_odom_bias
{

/// 状态机 (详设 4.7)
enum class BiasState : uint8_t
{
    UNINITIALIZED = 0,
    INITIALIZING = 1,
    TRACKING = 2,
    STALE = 3
};

/// 参数 (详设 3.3 参数表, 默认值为设计初值)
struct BiasEstimatorParams
{
    double gate_trans_threshold{0.3};       // m, 单帧观测相对 raw 的平移突变阈值
    double gate_yaw_threshold{0.17};        // rad (10°), yaw 突变阈值
    int gate_confirm_frames{3};             // 突变被采纳所需连续一致帧数
    int init_confirm_frames{3};             // 初始化需连续一致的观测帧数
    double absorb_time_constant{5.0};       // s, ctrl 向 raw 收敛时间常数 τ
    double max_correction_rate_trans{0.2};  // m/s, ctrl 平移修正速率硬上限
    double max_correction_rate_yaw{0.17};   // rad/s, ctrl yaw 修正速率硬上限
    // ---- cmd 快通道 (B 方案出口, 初值待仿真台扫 τ_f × 钳位定版) ----
    double cmd_absorb_time_constant{0.5};   // s, cmd 向 raw 收敛快时间常数 τ_f
                                            // (E7: τ0.5 治欠阻尼; →0 即 A 方案 raw 直出)
    double cmd_max_correction_rate_trans{0.5};  // m/s, cmd 平移修正速率上限
                                            // (需 > 漂移率 5~10cm/s, 否则重蹈
                                            // ctrl 0.2 顶死饱和的欠账)
    double cmd_max_correction_rate_yaw{0.5};    // rad/s, cmd yaw 修正速率上限
    double observation_timeout{2.0};        // s, 无有效观测超时进 STALE
                                            // ("有效"=被账本采纳, 详设 v2.1;
                                            // 误检风暴下账本停更同样进 STALE)
    int raw_median_window{1};               // raw 中值去毛刺窗口, 1=关闭
                                            // (默认不滤保持"监控要真")
    double max_tick_dt{0.2};                // s, 单拍步进 dt 防御上限 (定时器
                                            // 挂起恢复时防单步大跳, 实现防御项)
};

class BiasEstimator
{
public:
    explicit BiasEstimator(const BiasEstimatorParams & params);

    /**
     * @brief 喂入一帧 4DoF 偏差观测 (pose_math::bias_observation 的输出)
     * @param obs 观测到的 T_map_odom (4DoF)
     * @param t   观测时刻 (图像戳), 本地 ROS 钟, 秒
     *
     * 状态机行为:
     *   UNINITIALIZED → 进入 INITIALIZING (obs 为首个初始化候选)
     *   INITIALIZING  → 与前一候选一致则累积, 满 init_confirm_frames 帧
     *                   → raw = ctrl = 候选均值, 进 TRACKING (初始化不走慢吸收)
     *   TRACKING      → 门控判定 (详设 4.4)
     *   STALE         → 走门控判定, 观测被采纳 (正常路径或候选确认) 后回
     *                   TRACKING (详设 4.7: 恢复走门控, 长断流后的漂移偏差
     *                   按候选路径数帧确认, 正好是期望行为)
     */
    void add_observation(const pose_math::Transform4D & obs, double t);

    /**
     * @brief 定时步进 (tf_publish_rate 驱动): STALE 判定 + ctrl/cmd 双通道吸收
     * @param t_now 当前时刻, 要求单调不减
     *
     * 吸收律 (详设 4.5, ctrl/cmd 同律不同参): alpha = dt/τ;
     * delta = raw − state (yaw 最短角); 步进量 clamp —— 平移按向量范数
     * 限幅 ≤ rate_trans·dt (保方向缩模长), yaw 独立限幅 ≤ rate_yaw·dt。
     * ctrl 用 absorb_time_constant / max_correction_rate_*;
     * cmd 用 cmd_absorb_time_constant / cmd_max_correction_rate_* (快通道)。
     * 仅 TRACKING 状态步进; STALE 双通道冻结 (断流时变换保持最后估计)。
     */
    void tick(double t_now);

    /**
     * @brief EKF reset 瞬跳补偿 (详设 4.6)
     * @param d odom 系改写左乘变换 D 的 4DoF 表示 (已转 ENU):
     *          位置 reset → D = [I, Δp]; 航向 reset →
     *          D = [Rz(Δψ), (I−Rz(Δψ))·p_ob]; 由节点薄壳构造
     *
     * raw ← raw·D⁻¹, ctrl ← ctrl·D⁻¹, cmd ← cmd·D⁻¹ (同一 D, 三状态
     * 一致补偿); 门控候选队列清空 (旧坐标系下算出的偏差作废)。
     * TRACKING/STALE 均执行; INITIALIZING 清空初始化候选重新累积;
     * UNINITIALIZED 无内部状态可补偿, 仅计数。
     */
    void apply_reset(const pose_math::Transform4D & d);

    BiasState state() const { return state_; }
    const pose_math::Transform4D & raw() const { return raw_; }
    const pose_math::Transform4D & ctrl() const { return ctrl_; }
    const pose_math::Transform4D & cmd() const { return cmd_; }

    // ---- 健康信号数据源 (详设 4.8) ----
    /// raw 与 ctrl 的平移差范数
    double divergence_trans() const;
    /// raw 与 ctrl 的 yaw 差 (最短角, 绝对值)
    double divergence_yaw() const;
    /// cmd 与 ctrl 的平移差范数 = 出口快通道当前修正量 (B 作用量监控)
    double divergence_cmd_trans() const;
    /// cmd 与 ctrl 的 yaw 差 (最短角, 绝对值)
    double divergence_cmd_yaw() const;
    /// 最后一帧被账本采纳的观测时刻 (obs_age = now − last_obs_time);
    /// 被门控拒绝的观测不刷新 —— 误检风暴下账本停更, obs_age 增长直至
    /// STALE 告警 (详设 3.3/4.8 "有效观测" 口径, 对抗复核 F02)
    double last_obs_time() const { return last_obs_time_; }
    bool has_observation() const { return last_obs_time_ >= 0.0; }
    /// 门控拒绝累计: 候选队列被抛弃时按帧数计入 (被正常路径清空 / 与新帧
    /// 不一致被重置); 最终确认为真实跳变的候选帧不计 —— 合法重定位事件
    /// 不污染误检信号 (对抗复核 F13)
    uint32_t gate_reject_count() const { return gate_reject_count_; }
    /// 非有限 (NaN/inf) 观测丢弃计数
    uint32_t invalid_obs_count() const { return invalid_obs_count_; }
    /// 确认跳变事件
    uint32_t jump_count() const { return jump_count_; }
    double last_jump_trans() const { return last_jump_trans_; }
    double last_jump_yaw() const { return last_jump_yaw_; }
    /// EKF reset 补偿事件
    uint32_t reset_event_count() const { return reset_event_count_; }
    double last_reset_trans() const { return last_reset_trans_; }
    double last_reset_yaw() const { return last_reset_yaw_; }

private:
    /// 两帧 4DoF 的接近性判定 (平移范数与 yaw 最短角差均小于给定门限)
    static bool within(const pose_math::Transform4D & a,
                       const pose_math::Transform4D & b,
                       double trans_th, double yaw_th);
    /// 候选均值 (平移逐轴; yaw 以首帧为基准的相对角均值, 防环绕)
    static pose_math::Transform4D mean_of(
        const std::vector<pose_math::Transform4D> & c);
    /// 每通道独立中值 (yaw 以首元素为基准的相对角中值, 防环绕)
    static pose_math::Transform4D median_of(
        const std::deque<pose_math::Transform4D> & w);

    void gate_and_update(const pose_math::Transform4D & obs, double t);
    /// 正常路径采纳: raw_median_window > 1 时经中值窗口去毛刺
    void accept_raw(const pose_math::Transform4D & obs);
    /// raw 基准跳变 (跳变确认/初始化/reset 补偿) 后重置中值窗口
    void reseed_raw_window();
    void record_jump(const pose_math::Transform4D & old_raw,
                     const pose_math::Transform4D & new_raw);

    BiasEstimatorParams params_;
    BiasState state_{BiasState::UNINITIALIZED};

    /// 单通道吸收步进 (ctrl/cmd 共用律): state += clamp(delta·dt/τ)
    static void absorb_toward(const pose_math::Transform4D & target,
                              pose_math::Transform4D & state, double dt,
                              double tau, double rate_trans, double rate_yaw);

    pose_math::Transform4D raw_;
    pose_math::Transform4D ctrl_;
    pose_math::Transform4D cmd_;

    std::vector<pose_math::Transform4D> init_candidates_;
    std::vector<pose_math::Transform4D> gate_candidates_;
    std::deque<pose_math::Transform4D> raw_window_;    // 中值去毛刺窗口

    double last_obs_time_{-1.0};
    double last_tick_time_{-1.0};

    uint32_t gate_reject_count_{0};
    uint32_t invalid_obs_count_{0};
    uint32_t jump_count_{0};
    double last_jump_trans_{0.0};
    double last_jump_yaw_{0.0};
    uint32_t reset_event_count_{0};
    double last_reset_trans_{0.0};
    double last_reset_yaw_{0.0};
};

}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__CORE__BIAS_ESTIMATOR_HPP_

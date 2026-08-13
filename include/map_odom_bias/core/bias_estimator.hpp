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
#include <vector>

#include "map_odom_bias/core/pose_math.hpp"
#include "map_odom_bias/core/time_types.hpp"

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
    // (raw_median_window 已迁出: 中值去毛刺 v2 起由 obs_intake 链上的
    //  MedianWindow processor 承担, 位于门控前 —— 设计文档 v1 5.2)
    double max_tick_dt{0.2};                // s, 单拍步进 dt 防御上限 (定时器
                                            // 挂起恢复时防单步大跳, 实现防御项)
    // ---- v2 质量链: 三带门限 + 软降权 (设计文档 v1 4.4/5.3) ----
    double gate_soft_trans_threshold{-1.0}; // m, 正常吸收带上界 Δ; <0 =
                                            // 跟随 gate_trans_threshold
                                            // (降权带宽零 = 三带退化两带)
    double gate_soft_yaw_threshold{-1.0};   // rad, yaw 同义
    double band_quality{1.0 / 3.0};         // 降权带 (Δ<err≤L) 帧的质量因子;
                                            // MRS R×c≈τ×√c 直觉: 1/3 ≈ τ×3
};

class BiasEstimator
{
public:
    /**
     * @throw std::invalid_argument 质量链参数非法时 (Δ>L 或 band_quality
     *        出 (0,1]) —— 加载期硬断言, MRS rtk 配置把钳位层静默架空
     *        (limit==max) 的反例对治: 配置错误 fail-fast 不带病运行
     */
    explicit BiasEstimator(const BiasEstimatorParams & params);

    /**
     * @brief 喂入一帧 4DoF 偏差观测 (pose_math::bias_observation 的输出)
     * @param obs       观测到的 T_map_odom (4DoF)
     * @param t_sample  观测采样刻 (图像曝光戳, 采样钟域) —— 进账本/门控
     * @param t_arrival 本帧到达/处理刻 (节点钟) —— 判存活 (STALE 超时以
     *                  到达刻计, ⑤钟域纪律"进估计用采样戳、判存活用到达
     *                  刻": 深延迟链路的健康观测流不再被误判断流)
     * @param p_ob 本帧机体在 odom 系的位置 (bias_observation 的 odom_base
     *             位置) —— 门控在该点做机体点残差度量 (③修法, 设计文档 v1
     *             四节): yaw 解算噪声经力臂 |p_ob| 的平移伪差在此度量下
     *             恒零, 真平移误差如实计量。默认原点 = 参数空间旧度量。
     * @param quality 上游源质量 [0,1] (obs_intake 链 TagQuality 类 processor
     *             产出, 缺省满质量)。三带门控 (设计 4.4): err≤Δ 正常带
     *             q_band=1 / Δ<err≤L 降权带 q_band=band_quality / err>L
     *             拒绝带走候选队列。采纳帧记 q_eff = quality·q_band,
     *             消费在 tick 吸收 α←α·q_eff (5.3 软降权); **raw 账本
     *             始终采纳原值不降权 —— 打标消费分离, 账本纯净性保住
     *             判卷工具链**。
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
    void add_observation(const pose_math::Transform4D & obs,
                         SampleTime t_sample, HostTime t_arrival,
                         const std::array<double, 3> & p_ob = {{0.0, 0.0, 0.0}},
                         double quality = 1.0);

    /**
     * @brief 定时步进 (tf_publish_rate 驱动): STALE 判定 + ctrl/cmd 双通道吸收
     * @param t_now 当前到达/处理刻 (节点钟), 要求单调不减
     * @param eval_points 机体评估点集 (通常 = OdomBuffer 最新样本位置);
     *        钳位预算在此点集度量 —— 物理语义为"机体 setpoint 被拉动的
     *        速率 ≤ rate_trans" (③修法, yaw 步进经力臂的拉动计入同一预算)。
     *        空集退化为原点评估 = 参数空间旧钳位 (缓冲空的回退路径)。
     *
     * 吸收律 (详设 4.5, ctrl/cmd 同律不同参): alpha = dt/τ;
     * delta = raw − state (yaw 最短角); 步进量 clamp —— yaw 先独立限幅
     * ≤ rate_yaw·dt, 再按机体点位移预算 ≤ rate_trans·dt 全通道等比缩步
     * (保方向缩模长)。
     * ctrl 用 absorb_time_constant / max_correction_rate_*;
     * cmd 用 cmd_absorb_time_constant / cmd_max_correction_rate_* (快通道)。
     * 仅 TRACKING 状态步进; STALE 双通道冻结 (断流时变换保持最后估计)。
     */
    void tick(HostTime t_now,
              const std::vector<std::array<double, 3>> & eval_points =
                  std::vector<std::array<double, 3>>());

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
    /// raw 与 ctrl 在评估点集上的位置分歧 (③修法: "感知位置与指令位置
    /// 在机体处的分歧", 分歧门消费的本义); 空集 = 参数空间平移差范数
    double divergence_trans(
        const std::vector<std::array<double, 3>> & eval_points =
            std::vector<std::array<double, 3>>()) const;
    /// raw 与 ctrl 的 yaw 差 (最短角, 绝对值; 不吃力臂, 无评估点)
    double divergence_yaw() const;
    /// cmd 与 ctrl 在评估点集上的位置分歧 = 出口快通道当前修正量
    /// (B 作用量监控); 空集 = 参数空间平移差范数
    double divergence_cmd_trans(
        const std::vector<std::array<double, 3>> & eval_points =
            std::vector<std::array<double, 3>>()) const;
    /// cmd 与 ctrl 的 yaw 差 (最短角, 绝对值)
    double divergence_cmd_yaw() const;
    /// 最后被账本采纳观测的采样刻 (账本时间语义); 被门控拒绝的观测
    /// 不刷新 —— 误检风暴下账本停更, obs_age 增长直至 STALE 告警
    /// (详设 3.3/4.8 "有效观测" 口径, 对抗复核 F02)
    SampleTime last_obs_sample_time() const { return last_obs_sample_; }
    /// 最后被采纳观测的到达刻 (STALE 超时与 obs_age 的判定钟)
    HostTime last_obs_arrival() const { return last_obs_arrival_; }
    /// 最后被采纳观测的端到端延迟 (到达 − 采样, 同源钟假设;
    /// status.obs_delay 数据源 —— EV_DELAY 类旋钮不再离线反推)
    double last_obs_delay() const { return last_obs_delay_; }
    /// 最后被采纳观测的有效质量 q_eff = 源质量 × 带质量 (v2 软降权;
    /// 1.0 = 满质量满速吸收, 被拒绝的观测不刷新)
    double last_obs_quality() const { return current_quality_; }
    bool has_observation() const { return has_observation_; }
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
    /// 参考系事件单调计数 (④ResetEvent 的 iteration 数据源): 跳变确认
    /// (SOURCE_JUMP) 与 reset 补偿 (EKF_RESET) 均自增; 下游与出口数据
    /// 同拍比对上一拍即检测, 幂等且丢帧可补
    uint32_t reference_iteration() const { return reference_iteration_; }
    /// EKF reset 补偿事件
    uint32_t reset_event_count() const { return reset_event_count_; }
    double last_reset_trans() const { return last_reset_trans_; }
    double last_reset_yaw() const { return last_reset_yaw_; }

private:
    /// 两帧 4DoF 的接近性判定: 在机体点 p_ob 处的位置残差与 yaw 最短角差
    /// 均小于给定门限 (③修法: 度量空间 = 机体位置误差空间)
    static bool within_at(const pose_math::Transform4D & a,
                          const pose_math::Transform4D & b,
                          const std::array<double, 3> & p_ob,
                          double trans_th, double yaw_th);
    /// 候选均值 (平移逐轴; yaw 以首帧为基准的相对角均值, 防环绕)
    static pose_math::Transform4D mean_of(
        const std::vector<pose_math::Transform4D> & c);

    void gate_and_update(const pose_math::Transform4D & obs,
                         SampleTime t_sample, HostTime t_arrival,
                         const std::array<double, 3> & p_ob, double quality);
    /// 观测被账本采纳时的时序记账 (采样刻 + 到达刻 + 延迟)
    void note_observation(SampleTime t_sample, HostTime t_arrival);
    void record_jump(const pose_math::Transform4D & old_raw,
                     const pose_math::Transform4D & new_raw);

    BiasEstimatorParams params_;
    BiasState state_{BiasState::UNINITIALIZED};

    /// 单通道吸收步进 (ctrl/cmd 共用律): state += clamp(delta·(dt/τ)·q),
    /// 平移预算在 eval_points 上以机体点位移度量。quality = 软降权因子
    /// (5.3): 降权是线性缩放保留幅值信息 (零均值毛刺自抵消), 钳位是硬
    /// 非线性在饱和区丢幅值留符号 —— 降权在前、钳位退化为最后兜底
    static void absorb_toward(const pose_math::Transform4D & target,
                              pose_math::Transform4D & state, double dt,
                              double tau, double rate_trans, double rate_yaw,
                              const std::vector<std::array<double, 3>> & eval_points,
                              double quality);

    pose_math::Transform4D raw_;
    pose_math::Transform4D ctrl_;
    pose_math::Transform4D cmd_;

    std::vector<pose_math::Transform4D> init_candidates_;
    std::vector<pose_math::Transform4D> gate_candidates_;

    SampleTime last_obs_sample_;
    HostTime last_obs_arrival_;
    double last_obs_delay_{0.0};
    double current_quality_{1.0};    // 最后采纳帧 q_eff (软降权消费源)
    bool has_observation_{false};
    HostTime last_tick_;
    bool has_ticked_{false};

    uint32_t gate_reject_count_{0};
    uint32_t invalid_obs_count_{0};
    uint32_t jump_count_{0};
    double last_jump_trans_{0.0};
    double last_jump_yaw_{0.0};
    uint32_t reference_iteration_{0};
    uint32_t reset_event_count_{0};
    double last_reset_trans_{0.0};
    double last_reset_yaw_{0.0};
};

}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__CORE__BIAS_ESTIMATOR_HPP_

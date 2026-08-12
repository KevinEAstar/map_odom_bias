/**
 * @file map_odom_bias_node.cpp
 * @brief MapOdomBiasNode 实现 (薄壳: 消息进出 + 参数 + 定时器, 无算法逻辑)
 *
 * [✅] Step 1: 参数装载 (详设 3.3 参数表) + 核心件构造
 * [✅] Step 2: 订阅 —— 观测/odom 两路 + Px4ResetSource 接入层
 * [✅] Step 3: 发布 —— ctrl(topic+TF)/cmd/raw/pose_map_raw/status
 * [✅] Step 4: reset 补偿动作 (施加 + 缓冲静默 + 立即重发)
 * [✅] Step 5: 状态跳变日志与健康信号即时发布
 * [✅] Step 6: ③修法评估点接线 (门控=观测配对插值 / 钳位与 divergence
 *              = 缓冲最新样本, 空缓冲退化并计数)
 */

#include "map_odom_bias/ros/map_odom_bias_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

namespace map_odom_bias
{

namespace pm = pose_math;

using map_odom_bias::HostTime;
using map_odom_bias::SampleTime;

namespace
{

/// geometry_msgs Pose → 纯逻辑层 Pose
pm::Pose pose_from_msg(const geometry_msgs::msg::Pose & msg)
{
    pm::Pose p;
    p.p = {{msg.position.x, msg.position.y, msg.position.z}};
    p.q.w = msg.orientation.w;
    p.q.x = msg.orientation.x;
    p.q.y = msg.orientation.y;
    p.q.z = msg.orientation.z;
    return p;
}

/// 4DoF 变换 → TransformStamped (map→odom)
geometry_msgs::msg::TransformStamped transform_to_msg(
    const pm::Transform4D & t, const rclcpp::Time & stamp)
{
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "odom";
    msg.transform.translation.x = t.x;
    msg.transform.translation.y = t.y;
    msg.transform.translation.z = t.z;
    const pm::Quat q = pm::quat_from_yaw(t.yaw);
    msg.transform.rotation.w = q.w;
    msg.transform.rotation.x = q.x;
    msg.transform.rotation.y = q.y;
    msg.transform.rotation.z = q.z;
    return msg;
}

const char * state_name(BiasState s)
{
    switch (s) {
        case BiasState::UNINITIALIZED: return "UNINITIALIZED";
        case BiasState::INITIALIZING: return "INITIALIZING";
        case BiasState::TRACKING: return "TRACKING";
        case BiasState::STALE: return "STALE";
    }
    return "?";
}

}  // namespace

MapOdomBiasNode::MapOdomBiasNode()
: rclcpp::Node("map_odom_bias_node")
{
    // [✅] Step 1: 参数装载 (默认值 = 详设 3.3 设计初值)
    const std::string pose_topic =
        this->declare_parameter<std::string>("pose_topic", "/aprilslam/pose");
    const std::string odometry_topic =
        this->declare_parameter<std::string>("odometry_topic",
                                             "/fc_bridge/odometry");
    double odom_buffer_duration =
        this->declare_parameter<double>("odom_buffer_duration", 4.0);
    double max_extrapolation =
        this->declare_parameter<double>("max_extrapolation", 0.05);
    reset_settle_duration_ =
        this->declare_parameter<double>("reset_settle_duration", 0.05);

    BiasEstimatorParams bp;
    bp.init_confirm_frames =
        this->declare_parameter<int>("init_confirm_frames", 3);
    bp.gate_trans_threshold =
        this->declare_parameter<double>("gate_trans_threshold", 0.3);
    bp.gate_yaw_threshold =
        this->declare_parameter<double>("gate_yaw_threshold", 0.17);
    bp.gate_confirm_frames =
        this->declare_parameter<int>("gate_confirm_frames", 3);
    bp.absorb_time_constant =
        this->declare_parameter<double>("absorb_time_constant", 5.0);
    bp.max_correction_rate_trans =
        this->declare_parameter<double>("max_correction_rate_trans", 0.2);
    bp.max_correction_rate_yaw =
        this->declare_parameter<double>("max_correction_rate_yaw", 0.17);
    bp.cmd_absorb_time_constant =
        this->declare_parameter<double>("cmd_absorb_time_constant", 0.5);
    bp.cmd_max_correction_rate_trans =
        this->declare_parameter<double>("cmd_max_correction_rate_trans", 0.5);
    bp.cmd_max_correction_rate_yaw =
        this->declare_parameter<double>("cmd_max_correction_rate_yaw", 0.5);
    bp.observation_timeout =
        this->declare_parameter<double>("observation_timeout", 2.0);
    bp.raw_median_window =
        this->declare_parameter<int>("raw_median_window", 1);
    double tf_publish_rate =
        this->declare_parameter<double>("tf_publish_rate", 50.0);

    // 参数合法性校验: 非法值会导致除零/状态毒化, 回退默认并告警 (F08)
    auto sanitize = [this](const char * name, double & v, bool positive,
                           double fallback) {
        const bool bad = !std::isfinite(v) || (positive ? v <= 0.0 : v < 0.0);
        if (bad) {
            RCLCPP_WARN(this->get_logger(),
                "⚠ [BIAS] 参数 %s=%.3f 非法, 回退默认 %.3f", name, v, fallback);
            v = fallback;
        }
    };
    sanitize("odom_buffer_duration", odom_buffer_duration, true, 4.0);
    sanitize("max_extrapolation", max_extrapolation, false, 0.05);
    sanitize("reset_settle_duration", reset_settle_duration_, false, 0.05);
    sanitize("gate_trans_threshold", bp.gate_trans_threshold, true, 0.3);
    sanitize("gate_yaw_threshold", bp.gate_yaw_threshold, true, 0.17);
    sanitize("absorb_time_constant", bp.absorb_time_constant, true, 5.0);
    sanitize("max_correction_rate_trans", bp.max_correction_rate_trans, true, 0.2);
    sanitize("max_correction_rate_yaw", bp.max_correction_rate_yaw, true, 0.17);
    sanitize("cmd_absorb_time_constant", bp.cmd_absorb_time_constant, true, 0.5);
    sanitize("cmd_max_correction_rate_trans",
             bp.cmd_max_correction_rate_trans, true, 0.5);
    sanitize("cmd_max_correction_rate_yaw",
             bp.cmd_max_correction_rate_yaw, true, 0.5);
    sanitize("observation_timeout", bp.observation_timeout, true, 2.0);
    sanitize("tf_publish_rate", tf_publish_rate, true, 50.0);
    bp.init_confirm_frames = std::max(1, bp.init_confirm_frames);
    bp.gate_confirm_frames = std::max(1, bp.gate_confirm_frames);
    bp.raw_median_window = std::max(1, bp.raw_median_window);

    odom_buffer_.reset(new OdomBuffer(odom_buffer_duration, max_extrapolation));
    estimator_.reset(new BiasEstimator(bp));
    // 修正源两票链 (五节 5.2, C1: 接口 v1 就位): v1 首发 = FiniteGuard
    intake_.add(std::unique_ptr<ObsProcessor>(new FiniteGuard));

    // [✅] Step 2: 订阅
    // map 系机体位姿观测 (定位节点输出, header.stamp = 图像曝光时刻)
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, rclcpp::SensorDataQoS(),
        std::bind(&MapOdomBiasNode::pose_callback, this,
                  std::placeholders::_1));
    // 主输入: EKF 融合位姿 ENU (fc_bridge 出向, 全速率), 入历史缓冲
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, rclcpp::SensorDataQoS(),
        std::bind(&MapOdomBiasNode::odometry_callback, this,
                  std::placeholders::_1));
    // 辅输入: EKF reset 事件接入层 (对 PX4 只读, 无任何入向发布)
    reset_source_.reset(new Px4ResetSource(
        *this, std::bind(&MapOdomBiasNode::handle_reset, this,
                         std::placeholders::_1)));
    // TODO(健康门): 观测健康订阅待定位节点选型定型后接入 (详设 3.1);
    // 初版观测质量门 = 固定门限门控, 协方差不参与 (详设 8 开放问题 1)

    // [✅] Step 3: 发布
    // 控制通道 (感知侧 ctrl + 指令侧 cmd 双通道):
    // 晚启动的订阅者在一个发布周期内拿到最新值
    ctrl_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "~/t_map_odom_ctrl", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "~/t_map_odom_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    raw_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "~/t_map_odom_raw", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    pose_map_raw_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/pose_map_raw", rclcpp::SensorDataQoS());
    status_pub_ = this->create_publisher<map_odom_bias::msg::BiasStatus>(
        "~/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    // 参考系事件 (D1 独立 topic): 事件流不可丢, reliable + 深度 10
    reset_event_pub_ = this->create_publisher<map_odom_bias::msg::ResetEvent>(
        "~/reset_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(*this));

    // create_timer (非 wall): 跟随节点时钟, use_sim_time 下随 /clock 步进,
    // bag 回放/仿真非实时工况保持确定性 (F07)
    tick_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(1.0 / tf_publish_rate),
        std::bind(&MapOdomBiasNode::tick_timer_callback, this));
    status_timer_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(1.0),
        std::bind(&MapOdomBiasNode::status_timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
        "🗺 [BIAS] 启动: pose=%s odom=%s buffer=%.1fs extrap=%.2fs "
        "gate=(%.2fm, %.2frad) τ=%.1fs rate_max=(%.2fm/s, %.2frad/s) | "
        "cmd快通道 τ_f=%.2fs rate_max=(%.2fm/s, %.2frad/s) | "
        "timeout=%.1fs pub=%.0fHz",
        pose_topic.c_str(), odometry_topic.c_str(),
        odom_buffer_duration, max_extrapolation, bp.gate_trans_threshold,
        bp.gate_yaw_threshold, bp.absorb_time_constant,
        bp.max_correction_rate_trans, bp.max_correction_rate_yaw,
        bp.cmd_absorb_time_constant, bp.cmd_max_correction_rate_trans,
        bp.cmd_max_correction_rate_yaw,
        bp.observation_timeout, tf_publish_rate);
}

std::vector<std::array<double, 3>> MapOdomBiasNode::eval_points() const
{
    std::vector<std::array<double, 3>> pts;
    if (!odom_buffer_->empty()) {
        pts.push_back(odom_buffer_->newest_pose().p);
    }
    return pts;
}

void MapOdomBiasNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const double t_img = rclcpp::Time(msg->header.stamp).seconds();

    // ① 按图像戳在 odom 缓冲中插值 → T_odom_base(t) (拒绝时缓冲内已计数)
    pm::Pose odom_base;
    const OdomBuffer::QueryResult r =
        odom_buffer_->query(SampleTime{t_img}, &odom_base);
    if (r != OdomBuffer::QueryResult::OK) {
        RCLCPP_DEBUG(this->get_logger(),
            "[BIAS] 观测时间对齐失败 (%s), t_img=%.3f buffer=[%.3f, %.3f]",
            r == OdomBuffer::QueryResult::TOO_NEW ? "too_new" : "too_old",
            t_img, odom_buffer_->oldest_time(), odom_buffer_->newest_time());
        return;
    }

    // ② 4DoF 偏差观测 → 修正源两票链 → ③ 门控与 raw 更新 (纯逻辑层)。
    // 门控评估点 = 本帧配对的机体 odom 位置 (③修法: yaw 噪声力臂贡献恒零);
    // 采样刻 = 图像曝光戳 (进账本/门控), 到达刻 = 节点钟 (判存活)
    GlobalPoseObservation gobs;
    gobs.t_sample = SampleTime{t_img};
    gobs.t_arrival = HostTime{this->now().seconds()};
    gobs.T_obs = pm::bias_observation(pose_from_msg(msg->pose.pose), odom_base);
    gobs.p_ob = odom_base.p;
    if (!intake_.run(gobs, *estimator_)) {
        return;    // 生死票废弃 (计数进 status.intake_dropped)
    }
    const BiasState prev = estimator_->state();
    const uint32_t jumps_before = estimator_->jump_count();
    const pm::Transform4D raw_before = estimator_->raw();
    estimator_->add_observation(gobs.T_obs, gobs.t_sample, gobs.t_arrival,
                                gobs.p_ob);
    handle_state_change(prev);
    if (estimator_->jump_count() != jumps_before) {
        RCLCPP_WARN(this->get_logger(),
            "⚡ [BIAS-JUMP] 确认跳变: Δtrans=%.3fm Δyaw=%.3frad (重定位事件)",
            estimator_->last_jump_trans(), estimator_->last_jump_yaw());
        // SOURCE_JUMP 事件: delta = map 系点重定基增量 T_new·T_old⁻¹
        publish_reset_event(
            map_odom_bias::msg::ResetEvent::CAUSE_SOURCE_JUMP,
            pm::compose(estimator_->raw(), pm::inverse(raw_before)),
            this->now());
        publish_status(this->now());
    }

    // raw 逐观测更新时发布 (详设 3.2)
    if (initialized()) {
        publish_raw(msg->header.stamp);
    }
}

void MapOdomBiasNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    OdomSample s;
    s.t = SampleTime{rclcpp::Time(msg->header.stamp).seconds()};
    s.pose = pose_from_msg(msg->pose.pose);
    // 被缓冲拒绝的样本 (非有限/乱序/reset 静默窗口内的旧坐标系尾巴) 同样
    // 不得用于 pose_map_raw —— 否则 reset 后会把新系 raw 与旧系 odom 拼出
    // 跳变位姿喂给录制侧 (F03)
    if (!odom_buffer_->push(s)) {
        return;
    }

    // map 系机体位姿 (raw ⊗ 最新 odom 位姿), 随 odom 全速率, 录制侧消费
    if (initialized()) {
        const pm::Pose map_pose = pm::apply_to_pose(estimator_->raw(), s.pose);
        geometry_msgs::msg::PoseStamped out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = "map";
        out.pose.position.x = map_pose.p[0];
        out.pose.position.y = map_pose.p[1];
        out.pose.position.z = map_pose.p[2];
        out.pose.orientation.w = map_pose.q.w;
        out.pose.orientation.x = map_pose.q.x;
        out.pose.orientation.y = map_pose.q.y;
        out.pose.orientation.z = map_pose.q.z;
        pose_map_raw_pub_->publish(out);
    }
}

/**
 * [✅] Step 4: reset 补偿动作 (检测在 Px4ResetSource 接入层)
 */
void MapOdomBiasNode::handle_reset(const pm::Transform4D & d_total)
{
    const rclcpp::Time now = this->now();
    const BiasState prev = estimator_->state();
    estimator_->apply_reset(d_total);
    // 附带动作: 缓冲清空 + 静默窗口 (旧坐标系样本 + DDS 乱序尾巴防护)
    // 静默窗以样本戳判定 (同源钟假设在此显式声明, ⑤钟域)
    odom_buffer_->clear_and_settle(as_sample_time_assuming_same_clock(
        HostTime{now.seconds() + reset_settle_duration_}));

    intake_.reset_all();    // 链上历史属旧坐标系, 一并清空

    RCLCPP_WARN(this->get_logger(),
        "⚡ [BIAS-RESET] reset 补偿施加: D=(%.3f, %.3f, %.3f, %.3frad), "
        "state=%s",
        d_total.x, d_total.y, d_total.z, d_total.yaw, state_name(prev));
    // EKF_RESET 事件: delta = odom 系改写左乘增量 D (ENU)
    publish_reset_event(map_odom_bias::msg::ResetEvent::CAUSE_EKF_RESET,
                        d_total, now);
    // 补偿改写了 ctrl/cmd/raw, 立即重发 —— 否则下游在一个 tick 周期内
    // 持有旧坐标系的 map→odom 变换 (F09)
    if (initialized()) {
        publish_control_transforms(now);
        publish_raw(now);
    }
    publish_status(now);
}

void MapOdomBiasNode::tick_timer_callback()
{
    const rclcpp::Time now = this->now();
    const BiasState prev = estimator_->state();
    // ④ ctrl/cmd ← 各自 α·(raw − ·); 钳位预算在机体评估点度量 (③修法)。
    // 缓冲空 (reset 静默窗/断流) → 空集退化参数空间度量, 计数供健康信号
    const auto pts = eval_points();
    if (pts.empty() && initialized()) {
        ++eval_fallback_count_;
    }
    estimator_->tick(HostTime{now.seconds()}, pts);
    handle_state_change(prev);

    // UNINITIALIZED/INITIALIZING 不发布 (map 系不可用);
    // TRACKING/STALE 持续发布 —— 断流时保持最后估计, 可用性语义由健康信号承载
    if (initialized()) {
        publish_control_transforms(now);
    }
}

void MapOdomBiasNode::status_timer_callback()
{
    publish_status(this->now());
}

void MapOdomBiasNode::publish_control_transforms(const rclcpp::Time & stamp)
{
    // ctrl 与 cmd 同拍同 stamp: 下游 (navigation) 各自入小历史后按 position
    // 时戳配对选版, stamp 相等保证选出的 ctrl/cmd 天然同代 (跨 EKF reset
    // 不会出现半新半旧的字典对)
    const auto ctrl_msg = transform_to_msg(estimator_->ctrl(), stamp);
    ctrl_pub_->publish(ctrl_msg);
    tf_broadcaster_->sendTransform(ctrl_msg);    // 与 topic 同拍, 内容恒等
    cmd_pub_->publish(transform_to_msg(estimator_->cmd(), stamp));
}

void MapOdomBiasNode::publish_reset_event(
    uint8_t cause, const pm::Transform4D & delta, const rclcpp::Time & stamp)
{
    map_odom_bias::msg::ResetEvent msg;
    msg.header.stamp = stamp;
    msg.cause = cause;
    msg.iteration = estimator_->reference_iteration();
    msg.dx = delta.x;
    msg.dy = delta.y;
    msg.dz = delta.z;
    msg.dyaw = delta.yaw;
    reset_event_pub_->publish(msg);
}

void MapOdomBiasNode::publish_raw(const rclcpp::Time & stamp)
{
    raw_pub_->publish(transform_to_msg(estimator_->raw(), stamp));
}

void MapOdomBiasNode::publish_status(const rclcpp::Time & stamp)
{
    // divergence 与钳位同一评估点口径 (③修法): 机体处的位置分歧
    const auto pts = eval_points();
    map_odom_bias::msg::BiasStatus msg;
    msg.header.stamp = stamp;
    msg.state = static_cast<uint8_t>(estimator_->state());
    msg.raw_ctrl_divergence_trans = estimator_->divergence_trans(pts);
    msg.raw_ctrl_divergence_yaw = estimator_->divergence_yaw();
    msg.cmd_ctrl_divergence_trans = estimator_->divergence_cmd_trans(pts);
    msg.cmd_ctrl_divergence_yaw = estimator_->divergence_cmd_yaw();
    // obs_age 以到达刻计 (⑤钟域: 判存活用到达刻, 与 STALE 同口径)
    msg.obs_age = estimator_->has_observation()
        ? age_of(estimator_->last_obs_arrival(), HostTime{stamp.seconds()})
        : -1.0;
    msg.obs_delay = estimator_->last_obs_delay();
    msg.iteration = estimator_->reference_iteration();
    msg.gate_reject_count = estimator_->gate_reject_count();
    msg.intake_dropped = intake_.dropped_count();
    msg.obs_too_old = odom_buffer_->too_old_count();
    msg.obs_too_new = odom_buffer_->too_new_count();
    msg.eval_fallback_count = eval_fallback_count_;
    msg.last_jump_trans = estimator_->last_jump_trans();
    msg.last_jump_yaw = estimator_->last_jump_yaw();
    msg.reset_event_count = estimator_->reset_event_count();
    msg.last_reset_trans = estimator_->last_reset_trans();
    msg.last_reset_yaw = estimator_->last_reset_yaw();
    status_pub_->publish(msg);
}

void MapOdomBiasNode::handle_state_change(BiasState prev_state)
{
    const BiasState cur = estimator_->state();
    if (cur == prev_state) {
        return;
    }
    switch (cur) {
        case BiasState::INITIALIZING:
            RCLCPP_INFO(this->get_logger(),
                "⏳ [BIAS-INIT] 收到首个有效观测, 初始化确认中");
            break;
        case BiasState::TRACKING:
            if (prev_state == BiasState::INITIALIZING) {
                RCLCPP_INFO(this->get_logger(),
                    "✅ [BIAS-INIT] 初始化完成: bias=(%.3f, %.3f, %.3f, %.3frad)",
                    estimator_->raw().x, estimator_->raw().y,
                    estimator_->raw().z, estimator_->raw().yaw);
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "✳ [BIAS-RESUME] 观测恢复, 回 TRACKING");
            }
            break;
        case BiasState::STALE:
            RCLCPP_WARN(this->get_logger(),
                "🕳 [BIAS-STALE] 观测断流, 双状态冻结 (变换保持最后估计, "
                "系统退化为纯里程计飞行)");
            break;
        case BiasState::UNINITIALIZED:
            break;
    }
    publish_status(this->now());    // 状态跳变时立即发布 (详设 3.2)
}

}  // namespace map_odom_bias

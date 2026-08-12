/**
 * @file map_odom_bias_node.hpp
 * @brief map→odom 偏置层节点薄壳: 消息进出 / 参数装载 / 定时器调度,
 *        自身无算法逻辑 (迁自 drone_tf_manager tf_manager_node d3bb4ac)
 *
 * 职责: 维护 map→odom 变换的 raw/ctrl/cmd 三状态并发布。
 * 核心算法在 OdomBuffer / BiasEstimator (纯逻辑类), 本节点仅:
 *   - 订阅 pose_topic (map 系位姿观测) + odometry_topic (odom 历史)
 *     + EKF reset 事件 (Px4ResetSource 接入层, 对飞控零入向)
 *   - 发布 ~/t_map_odom_ctrl (控制感知侧) + ~/t_map_odom_cmd (控制指令侧,
 *     快通道, 与 ctrl 同拍同 stamp —— 下游按 stamp 配对天然同代)
 *     + TF map→odom (调试/可视化, 内容 = ctrl)
 *     + ~/t_map_odom_raw (监控) + ~/pose_map_raw (录制侧) + ~/status (健康)
 *   - ③修法接线: 门控评估点 = 观测配对的 odom 插值位置; 钳位/divergence
 *     评估点 = odom 缓冲最新样本位置 (缓冲空退化参数空间度量并计数)
 *
 * 边界 (红线继承): 不发布任何发往飞控的数据; 不在线重置任何模块状态;
 * 异常只发健康信号, 降级决策交由上层。
 */

#ifndef MAP_ODOM_BIAS__ROS__MAP_ODOM_BIAS_NODE_HPP_
#define MAP_ODOM_BIAS__ROS__MAP_ODOM_BIAS_NODE_HPP_

#include <array>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <map_odom_bias/msg/bias_status.hpp>
#include <map_odom_bias/msg/reset_event.hpp>

#include "map_odom_bias/core/bias_estimator.hpp"
#include "map_odom_bias/core/obs_intake.hpp"
#include "map_odom_bias/core/odom_buffer.hpp"
#include "map_odom_bias/ros/px4_reset_source.hpp"

namespace map_odom_bias
{

class MapOdomBiasNode : public rclcpp::Node
{
public:
    MapOdomBiasNode();

private:
    // ---- 订阅回调 ----
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    /// ResetSource 窄接口回调: 补偿施加 + 缓冲清空静默 + 立即重发
    void handle_reset(const pose_math::Transform4D & d_total);

    // ---- 定时器 ----
    void tick_timer_callback();      // tf_publish_rate: ctrl/cmd 收敛步进 + 发布
    void status_timer_callback();    // 1 Hz 健康信号

    // ---- 发布 ----
    /// ctrl + cmd + TF 同拍发布 (同一 stamp, 下游配对同代的保证)
    void publish_control_transforms(const rclcpp::Time & stamp);
    void publish_raw(const rclcpp::Time & stamp);
    void publish_status(const rclcpp::Time & stamp);
    /// 参考系事件发布 (D1 独立 topic; iteration 取估计器权威计数)
    void publish_reset_event(uint8_t cause, const pose_math::Transform4D & delta,
                             const rclcpp::Time & stamp);
    /// 状态机跳变检测: 日志分级打印 + status 立即发布
    void handle_state_change(BiasState prev_state);

    /// 机体评估点集 (③修法): odom 缓冲最新样本位置; 空缓冲返回空集
    /// (下游 transform_error 退化为参数空间度量)
    std::vector<std::array<double, 3>> eval_points() const;

    bool initialized() const
    {
        const BiasState s = estimator_->state();
        return s == BiasState::TRACKING || s == BiasState::STALE;
    }

    // ---- 参数 (详设 3.3) ----
    double reset_settle_duration_{0.05};

    // ---- 核心件 (参数装载后构造) ----
    std::unique_ptr<OdomBuffer> odom_buffer_;
    std::unique_ptr<BiasEstimator> estimator_;
    std::unique_ptr<Px4ResetSource> reset_source_;
    ObsIntake intake_;    // 修正源两票链 (v1 = FiniteGuard)

    uint32_t eval_fallback_count_{0};    // tick 时缓冲空的退化计数 (健康信号)

    // ---- ROS 接口 ----
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr ctrl_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_map_raw_pub_;
    rclcpp::Publisher<map_odom_bias::msg::BiasStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<map_odom_bias::msg::ResetEvent>::SharedPtr reset_event_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__ROS__MAP_ODOM_BIAS_NODE_HPP_

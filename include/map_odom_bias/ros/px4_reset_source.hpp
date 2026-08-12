/**
 * @file px4_reset_source.hpp
 * @brief PX4 EKF reset 事件接入层 (设计文档 v1 3.2: ROS 壳内可替换的
 *        ResetSource 适配点, PX4 实现随包提供)
 *
 * 职责: 订阅 /fmu/out/vehicle_local_position (对 PX4 只读, 无任何入向
 * 发布), 以三 reset counter 比较检测 EKF reset 事件, 把 delta 从 NED
 * 转 ENU 并合成 odom 系改写左乘增量 D_total (先位置后航向), 经窄接口
 * 回调交给节点 —— 节点与核心库不感知 PX4。
 *
 * 非 PX4 系统替换本类: 实现同签名回调注入即可 (ResetCallback 收到的
 * D_total 语义见 BiasEstimator::apply_reset)。
 */

#ifndef MAP_ODOM_BIAS__ROS__PX4_RESET_SOURCE_HPP_
#define MAP_ODOM_BIAS__ROS__PX4_RESET_SOURCE_HPP_

#include <array>
#include <cstdint>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "map_odom_bias/core/pose_math.hpp"

namespace map_odom_bias
{

class Px4ResetSource
{
public:
    /// reset 事件回调: D_total = odom 系改写左乘增量 (已转 ENU, 4DoF)
    using ResetCallback = std::function<void(const pose_math::Transform4D &)>;

    Px4ResetSource(rclcpp::Node & node, ResetCallback on_reset);

private:
    void local_position_callback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    rclcpp::Logger logger_;
    ResetCallback on_reset_;

    // ---- reset 检测状态 (模板: fc_bridge local_position_callback) ----
    bool first_lpos_received_{false};
    uint8_t last_xy_reset_counter_{0};
    uint8_t last_z_reset_counter_{0};
    uint8_t last_heading_reset_counter_{0};
    std::array<float, 3> last_lpos_ned_{{0.0f, 0.0f, 0.0f}};

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_;
};

}  // namespace map_odom_bias

#endif  // MAP_ODOM_BIAS__ROS__PX4_RESET_SOURCE_HPP_

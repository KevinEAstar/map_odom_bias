/**
 * @file px4_reset_source.cpp
 * @brief Px4ResetSource 实现: counter 比较检测 + delta NED→ENU + D 合成
 *
 * 逻辑迁自 drone_tf_manager tf_manager_node (d3bb4ac) 的
 * local_position_callback, 检测细节与告警口径逐位保持。
 */

#include "map_odom_bias/ros/px4_reset_source.hpp"

#include <cmath>

namespace map_odom_bias
{

namespace pm = pose_math;

namespace
{

// NED→ENU (坐标系铁律): ENU_x = NED_y, ENU_y = NED_x, ENU_z = −NED_z。
// 本包对 fc_bridge frame_transforms 的唯一消费点, 内联避免跨包依赖
// (设计文档 v1 8.2 裁决)
std::array<double, 3> ned_to_enu(double x_ned, double y_ned, double z_ned)
{
    return {{y_ned, x_ned, -z_ned}};
}

}  // namespace

Px4ResetSource::Px4ResetSource(rclcpp::Node & node, ResetCallback on_reset)
: logger_(node.get_logger()), on_reset_(std::move(on_reset))
{
    // 对接 PX4 报文 QoS: best_effort + volatile
    const auto px4_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    sub_ = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_qos,
        std::bind(&Px4ResetSource::local_position_callback, this,
                  std::placeholders::_1));
}

/**
 * EKF reset 检测 (详设 4.6; 模板 = fc_bridge 锚点 reset 同步)
 * reset 是坐标系事件而非观测误差: raw/ctrl 同步瞬跳补偿, 不走门控不走慢吸收。
 * delta 语义 = new − old (NED); 同帧多类并发按固定次序先位置后航向,
 * 合成 D_total = D_yaw · D_pos 一次施加 (依次施加的等价合成)。
 */
void Px4ResetSource::local_position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    if (!first_lpos_received_) {
        // 首帧只缓存 counter: 防节点晚启动把历史累计当作事件
        last_xy_reset_counter_ = msg->xy_reset_counter;
        last_z_reset_counter_ = msg->z_reset_counter;
        last_heading_reset_counter_ = msg->heading_reset_counter;
        last_lpos_ned_ = {{msg->x, msg->y, msg->z}};
        first_lpos_received_ = true;
        return;
    }

    const bool xy_reset = msg->xy_reset_counter != last_xy_reset_counter_;
    const bool z_reset = msg->z_reset_counter != last_z_reset_counter_;
    const bool heading_reset =
        msg->heading_reset_counter != last_heading_reset_counter_;

    if (xy_reset || z_reset || heading_reset) {
        // 有限性守卫: 非法 delta 一旦补进 raw/ctrl 会毒化控制变换且无法
        // 恢复; 跳过补偿但推进 counter, 防同一事件每帧重触发 (F05)
        const bool delta_finite =
            std::isfinite(msg->delta_xy[0]) && std::isfinite(msg->delta_xy[1]) &&
            std::isfinite(msg->delta_z) && std::isfinite(msg->delta_heading) &&
            std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z);
        if (!delta_finite) {
            RCLCPP_ERROR(logger_,
                "⚡ [EKF跳变] reset delta 含非有限值, 跳过补偿 (xy=%d z=%d "
                "heading=%d) —— 坐标系可能错位, 等待观测经门控重收敛",
                xy_reset, z_reset, heading_reset);
            last_xy_reset_counter_ = msg->xy_reset_counter;
            last_z_reset_counter_ = msg->z_reset_counter;
            last_heading_reset_counter_ = msg->heading_reset_counter;
            return;
        }

        pm::Transform4D d_total;    // 单位变换起步

        if (xy_reset || z_reset) {
            const float dx_ned = xy_reset ? msg->delta_xy[0] : 0.0f;
            const float dy_ned = xy_reset ? msg->delta_xy[1] : 0.0f;
            const float dz_ned = z_reset ? msg->delta_z : 0.0f;
            if (xy_reset) {
                // Sanity check (模板同款): new ≈ old + delta, 语义反了会触发
                const float ex = last_lpos_ned_[0] + dx_ned;
                const float ey = last_lpos_ned_[1] + dy_ned;
                const float err = std::sqrt((msg->x - ex) * (msg->x - ex) +
                                            (msg->y - ey) * (msg->y - ey));
                if (err > 0.05f) {
                    RCLCPP_ERROR(logger_,
                        "⚡ [EKF跳变] delta_xy 语义异常! 预期(%.2f, %.2f) vs "
                        "实际(%.2f, %.2f), err=%.2fm (只告警, 不拒绝补偿)",
                        ex, ey, msg->x, msg->y, err);
                }
            }
            // 位置 reset: D_pos = [I, Δp_enu]
            const auto dp_enu = ned_to_enu(dx_ned, dy_ned, dz_ned);
            d_total.x = dp_enu[0];
            d_total.y = dp_enu[1];
            d_total.z = dp_enu[2];
        }

        if (heading_reset) {
            // 航向 reset: odom 系绕机体当前位置旋转 (PX4 保持位置坐标不动),
            // Δψ_enu = −delta_heading (NED 航向 CW 与 ENU yaw CCW 互为反号);
            // D 的构造在 pose_math (纯逻辑, 有单测覆盖, F06)
            const double dpsi = -static_cast<double>(msg->delta_heading);
            const auto p_ob = ned_to_enu(msg->x, msg->y, msg->z);
            const pm::Transform4D d_yaw = pm::make_heading_reset_delta(
                dpsi, {{p_ob[0], p_ob[1], p_ob[2]}});
            d_total = pm::compose(d_yaw, d_total);
        }

        RCLCPP_WARN(logger_,
            "⚡ [EKF跳变] 检测到 reset (xy=%d z=%d heading=%d): "
            "D=(%.3f, %.3f, %.3f, %.3frad)",
            xy_reset, z_reset, heading_reset,
            d_total.x, d_total.y, d_total.z, d_total.yaw);
        on_reset_(d_total);

        last_xy_reset_counter_ = msg->xy_reset_counter;
        last_z_reset_counter_ = msg->z_reset_counter;
        last_heading_reset_counter_ = msg->heading_reset_counter;
    }

    last_lpos_ned_ = {{msg->x, msg->y, msg->z}};
}

}  // namespace map_odom_bias

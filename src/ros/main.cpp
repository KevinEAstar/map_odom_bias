/**
 * @file main.cpp
 * @brief map_odom_bias_node 入口
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "map_odom_bias/ros/map_odom_bias_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<map_odom_bias::MapOdomBiasNode>());
    rclcpp::shutdown();
    return 0;
}

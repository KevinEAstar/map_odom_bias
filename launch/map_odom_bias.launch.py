# map_odom_bias 启动: 加载参数文件, 支持 use_sim_time 覆盖 (SITL 联调)
# 并行对照切换: node_name:=tf_manager_node 即可冒充原节点 (topic 全 ~/ 相对)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('map_odom_bias'),
        'config', 'map_odom_bias_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('node_name', default_value='map_odom_bias_node'),
        Node(
            package='map_odom_bias',
            executable='map_odom_bias_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[
                params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])

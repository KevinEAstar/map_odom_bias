cmake_minimum_required(VERSION 3.8)
project(map_odom_bias)
# 不设置 C++17！ROS 2 Humble 默认用 C++14，与板端工具链保持一致
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# ===== 消息 =====
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BiasStatus.msg"
  DEPENDENCIES std_msgs
)
# ===== 纯逻辑核心库 (零 ROS 依赖, 单元测试与 ROS 壳共用) =====
add_library(map_odom_bias_core STATIC
  src/core/pose_math.cpp
  src/core/odom_buffer.cpp
  src/core/bias_estimator.cpp
)
target_include_directories(map_odom_bias_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ===== ROS 壳节点 =====
add_executable(map_odom_bias_node
  src/ros/main.cpp
  src/ros/map_odom_bias_node.cpp
  src/ros/px4_reset_source.cpp
)
ament_target_dependencies(map_odom_bias_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  px4_msgs
)
target_link_libraries(map_odom_bias_node map_odom_bias_core)
# 同包消息 typesupport (Humble 惯例)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(map_odom_bias_node "${cpp_typesupport_target}")
# ===== 安装与导出 =====
install(TARGETS map_odom_bias_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS map_odom_bias_core
  EXPORT export_map_odom_bias_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_map_odom_bias_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
install(DIRECTORY
  include/
  DESTINATION include
)
# ===== 测试 =====
if(BUILD_TESTING)
  # 独立仓根直接 colcon 构建, build/install 落在包目录内, ament_lint_auto
  # 会把生成物一并扫描产生海量误报 —— 本仓只跑 gtest, 不挂 lint_auto
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pose_math
    test/test_pose_math.cpp
  )
  target_link_libraries(test_pose_math map_odom_bias_core)

  ament_add_gtest(test_odom_buffer
    test/test_odom_buffer.cpp
  )
  target_link_libraries(test_odom_buffer map_odom_bias_core)

  ament_add_gtest(test_bias_estimator
    test/test_bias_estimator.cpp
  )
  target_link_libraries(test_bias_estimator map_odom_bias_core)
endif()

ament_package()

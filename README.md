# map_odom_bias

map→odom 偏置层：全局定位源（AprilTag / UWB / 动捕等）与里程计（EKF2 / VIO）之间的
通用修正中间层。维护 raw/ctrl/cmd 三状态偏置变换，以门控 + 双轨慢吸收 + EKF reset
补偿把"全局观测的跳变与噪声"整形为"控制侧可消费的平滑修正"。

## 定位

- **raw**：实测偏差账本，采纳观测原值不做吸收——监控要真。
- **ctrl**：感知侧消费（位置→map 垂足/进度），一阶低通 + 速率钳位慢速收敛——感知要稳。
- **cmd**：指令侧消费（map 系 setpoint→odom 系发布），同律独立快时间常数——指令要贴。

单一 EKF 只有一个增益旋钮，"贴观测"与"输出平滑"结构性互斥；双状态 + 双轨时间常数
是本包的结构核心（台架对照验证结论见来源仓库实验记录）。

## 分层

```
include/map_odom_bias/core/   纯逻辑核心 (零 ROS 依赖, 单测全覆盖)
    pose_math       4DoF 变换数学 + 偏差观测构造 + 机体点残差度量
    odom_buffer     odom 历史缓冲 + 时间戳对齐插值
    bias_estimator  状态机 + 门控 + 双轨吸收 + reset 补偿
include/map_odom_bias/ros/    ROS 2 壳 (1 节点)
    map_odom_bias_node   薄壳: 消息进出 / 参数装载 / 定时器, 无算法逻辑
    px4_reset_source     PX4 EKF reset 接入层 (可替换 ResetSource 形态,
                         px4_msgs 依赖隔离于此; 非 PX4 系统替换本类)
msg/BiasStatus.msg            健康信号 (状态机 / divergence / 各类计数)
include/map_odom_bias/tools/  消费者工具件 (header-only, 零 ROS 依赖)
    reference_tools 参考系载体类型二分: Anchored (带帧, 只经变换取值,
                    无叠加接口) / Rebasable (裸值, rebase(old,new) 端点
                    差分 + 相邻 delta 快捷路径仅 Point) + RebaseBus
                    (ResetEvent 统一分发, 幂等/跳号标记) + ResetTransaction
                    (跳变窗口判据关一拍, 语义性恢复)
```

工具件契约：消费侧持有的每个锚定量二选一——经变换出口取值的走
`Anchored`（禁叠加，参考系事件由出口吸收），自持裸值的走 `Rebasable`
（必须随 ResetEvent 补偿）；双重补偿在编译期写不出（static_assert 反例
见 test_reference_tools.cpp）。cause 过滤在总线 handler 侧声明。

度量口径（机体点残差）：门控在本帧观测配对的机体 odom 位置处评估残差，
吸收钳位与 divergence 在 odom 缓冲最新样本位置处评估——yaw 解算噪声经
力臂放大为平移伪差的路径全部关闭；缓冲空时退化为参数空间度量并计数。

## 构建与测试

ROS 2 Humble / C++14；需要 px4_msgs overlay（版本须匹配固件，由部署环境
提供，例如主项目工作空间 install）：

```bash
source /opt/ros/humble/setup.bash
source <px4_msgs_ws>/install/setup.bash
colcon build --packages-select map_odom_bias
colcon test --packages-select map_odom_bias && colcon test-result --verbose
ros2 launch map_odom_bias map_odom_bias.launch.py   # node_name:= 可冒充原节点
```

## 血统

核心逻辑源自 FlyerO1ROS2 `drone_tf_manager`（迁出基线 d3bb4ac，真机与台架验证过的
行为面），在本仓库独立演进；原节点保持在位作并行对照，回归判卷后切换。
设计依据：`坐标转换资产_设计文档_v1_2026_08_11`（含 MRS 机制精读与现状盘点）。

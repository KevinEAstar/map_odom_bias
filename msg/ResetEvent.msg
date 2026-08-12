# ResetEvent.msg
# 参考系事件 (设计文档 v1 六节, 裁决 D1 = 独立 topic 载体)
# iteration 与 status.iteration 同源双载: 下游与出口数据同拍比对上一拍
# 即检测参考系变化, 幂等且丢帧可补 —— 取代各节点自订 VehicleLocalPosition
# 重复检测的分散形态。
std_msgs/Header header

uint8 CAUSE_SOURCE_JUMP=0   # 观测跳变确认 (raw 账本重定位, 变换已吸收该跳)
uint8 CAUSE_EKF_RESET=1     # EKF reset 补偿 (变换与 odom 同跳)
uint8 CAUSE_MANUAL=2        # 人工干预 (预留)
uint8 cause

uint32 iteration            # 参考系事件单调计数 (估计器权威计数)

# delta: 4DoF 便利品, 仅供点类快捷路径消费; 语义随 cause 不同:
#   SOURCE_JUMP — map 系点重定基增量 T_raw_new · T_raw_old⁻¹
#   EKF_RESET   — odom 系改写左乘增量 D (已转 ENU)
# 结构化状态 (轨迹/索引/积分器) 禁用 delta, 应以自持旧值 + 本事件后
# 首帧新数据做 rebase(old, new) (设计文档 v1 6.1)。
float64 dx
float64 dy
float64 dz
float64 dyaw

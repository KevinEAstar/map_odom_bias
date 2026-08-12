# BiasStatus.msg
# map→odom 偏置层 (map_odom_bias) 健康信号
# 本节点只发布健康信号, 不做任何在线自动重置, 降级决策交由上层。
# map 系航点可执行性判据: 仅 TRACKING/STALE 可执行, STALE 伴随告警。
# divergence 度量在机体评估点 (odom 缓冲最新样本位置) 上计量 —— "感知
# 位置与指令位置在机体处的分歧"; 缓冲空时退化为参数空间平移差。
std_msgs/Header header

uint8 STATE_UNINITIALIZED=0
uint8 STATE_INITIALIZING=1
uint8 STATE_TRACKING=2
uint8 STATE_STALE=3
uint8 state

float64 raw_ctrl_divergence_trans  # raw 与 ctrl 在机体点的位置分歧 m; 持续偏大 = 漂移速率超出吸收带宽或观测质量异常
float64 raw_ctrl_divergence_yaw    # raw 与 ctrl 的 yaw 差 rad (最短角)
float64 cmd_ctrl_divergence_trans  # cmd 与 ctrl 在机体点的位置分歧 m = 出口快通道当前修正量 (作用量监控)
float64 cmd_ctrl_divergence_yaw    # cmd 与 ctrl 的 yaw 差 rad (最短角)
float64 obs_age                    # 最后有效观测距今 s (断流预警, 早于 STALE 触发); 未收到过观测 = -1
uint32 gate_reject_count           # 门控拒绝累计; 增长速率异常 = 误检率上升/双解翻转频发
uint32 obs_too_old                 # 时间戳对齐丢弃计数 (早于缓冲/缓冲空)
uint32 obs_too_new                 # 时间戳对齐丢弃计数 (超外推上限); 链路延迟/时钟异常线索
uint32 eval_fallback_count         # tick 时 odom 缓冲为空的累计 (钳位/divergence 退化为参数空间度量)
float64 last_jump_trans            # 最近一次确认跳变的平移幅度 m (重定位事件记录)
float64 last_jump_yaw              # 最近一次确认跳变的 yaw 幅度 rad
uint32 reset_event_count           # EKF reset 补偿事件累计; 频发 = EKF2 内部估计不稳
float64 last_reset_trans           # 最近一次 reset 的 delta 平移范数 m
float64 last_reset_yaw             # 最近一次 reset 的 delta yaw 幅度 rad

# 航点未跟随：新飞控源码核对

审查仓库：https://github.com/HuangGL03/XUSTBfly_control

固定提交：7a1528f15d8c16d8e6b0c2b94513ab3abeb45afd。只读取源码，没有修改或烧录飞控，也没有启动硬件通信。

## 已核实的控制链路

- application/mavlink.c:315 接收 ID84，目标系统/组件为1或广播0，frame为8或9，vx/vy均启用且数值有限才接受。电脑当前target=1/1、frame=8、mask=3559符合此条件。
- 接收值原样存入 pc_data.vx_set_mps / vy_set_mps，更新 velocity_update_ms 并增加 velocity_rx_count。
- application/Aircraft_ctrl_task.c:1203 只有 B=1、C=0 进入 Aircraft_FLOW_RUN。用户确认两次实验为此组合，可以排除模式选错。
- application/Aircraft_ctrl_task.c:1638 只有死区处理后的 Right_X 和 Right_Y 同时为0，才调用 Flow_Apply_PC_Velocity。任一右杆轴超出死区，将同时改用摇杆的X/Y速度。FLOW_RUN_STICK_DEADBAND=5，杆量范围约-100..100。
- algorithm/flow_config.h:27 的 FLOW_PC_VELOCITY_TIMEOUT_MS=300；超过有效期后保留摇杆给定，回中时为零。10Hz标称周期100ms满足要求，但不能证明无线接收间隔始终低于300ms。
- 光流无有效有限XY速度时，本机速度PID清零，目标滚转/俯仰为0。Auto模式使用ID82姿态目标，不使用ID84速度环。

## 待确认事项

1. 用户认为先前可能没有让右杆回中，准备保持当前代码和参数复测。这是尚未验证的原因，不宣称已定位故障。
2. 新源码的ID84与ID250都定义Y向右为正，速度环直接使用收到的vy。电脑配置position_hold.yaml仍为velocity_y_sign=-1，这是此前用户实际要求的适配。按新源码声明的物理方向，两端约定存在冲突。例如机头对准map+X，希望向map+Y（左）移动时，内部right应为负；取反后飞控却收到右向正值。尚未修改该实测参数，应另行核对当前固件的实际物理方向。
3. 旧日志没有飞控收到/采用ID84的记录，不能证明此前每一帧到达或执行。heartbeat当前custom_mode固定0，也不能用于回溯飞控当时的模式。
4. 两次旧日志的ID250接收间隔>300ms分别139次和154次；这是下行接收记录，不能当作上行ID84超时的证据。

## 下次实验可观察的数据

- 电脑：/fc/velocity_setpoint_body 为内部前/右速度，/fc/velocity_setpoint_sent 为成功交给本机UDP的打包速度，已包含发送端Y符号；新日志velocity_sent.csv保存后者。这不是飞控接收回执。
- 飞控调试器：pc_data.velocity_rx_count 是否持续增加；HAL_GetTick()-pc_data.velocity_update_ms 是否<=300；pc_data.vx_set_mps/vy_set_mps 是否等于电脑打包值。
- 飞控调试器：aircraft_control.remote_data->Right_X/Right_Y、aircraft_control.flow_data.control.flow_mode_active、xy_pid_active、vx_set_mps/vy_set_mps。实际采用的速度目标应在回中、光流有效、指令新鲜时匹配pc_data。
- 若目标值已被采用，再对照flow_data.vx_body_mps/vy_body_mps、control.pitch_target_deg/roll_target_deg及实际姿态，区分速度跟踪问题与通信/仲裁问题。

## 源码位置

- [ID84接收](https://github.com/HuangGL03/XUSTBfly_control/blob/7a1528f15d8c16d8e6b0c2b94513ab3abeb45afd/application/mavlink.c#L315)
- [300ms时效与速度选择](https://github.com/HuangGL03/XUSTBfly_control/blob/7a1528f15d8c16d8e6b0c2b94513ab3abeb45afd/application/Aircraft_ctrl_task.c#L1081)
- [右杆接管与速度PID](https://github.com/HuangGL03/XUSTBfly_control/blob/7a1528f15d8c16d8e6b0c2b94513ab3abeb45afd/application/Aircraft_ctrl_task.c#L1630)
- [速度、死区、超时参数](https://github.com/HuangGL03/XUSTBfly_control/blob/7a1528f15d8c16d8e6b0c2b94513ab3abeb45afd/algorithm/flow_config.h#L26)

# 飞控dfe1d83与电脑端的联调审查

审查对象为[HuangGL03/XUSTBfly_control dfe1d83](https://github.com/HuangGL03/XUSTBfly_control/commit/dfe1d83a748459e77c12378f6a98e1499daaf55b)。飞控由用户维护；本次只读取源代码并核对电脑接口，没有修改飞控或生成新的飞控补丁。

## 发现

### P1：默认QGC调试开关会关闭导航必需的yaw

flow_config.h:4的MAVLINK_QGC_FLOW_DEBUG_ENABLE=1；mavlink.c:573–576为互斥分支，启用时发送两路ID250而不调用send_yaw_debug。电脑只接受ID254 ind=1作为yaw，且不会将ID250速度误当航向。结果是/fc/attitude没有有效yaw、echo为0、角度桥接发送enabled=0，start_here无法完成。

ROS导航配置应将该开关设为0并重新构建/烧录，或由飞控维护端改成不阻断yaw的调试方式；保持现有小负载目标时优先关闭QGC调试。

源码：[flow_config.h:4](https://github.com/HuangGL03/XUSTBfly_control/blob/dfe1d83a748459e77c12378f6a98e1499daaf55b/algorithm/flow_config.h#L4)，[遥测分支](https://github.com/HuangGL03/XUSTBfly_control/blob/dfe1d83a748459e77c12378f6a98e1499daaf55b/application/mavlink.c#L570)。

### P2：echo只记录，排队的旧角度会被当作新目标

handle_pc_flow_extension保存flow_target_echo_ms，同时用HAL_GetTick刷新flow_target_update_ms。Flow_Apply_PC_Target只检查接收年龄，不检查echo。用提取的真实函数做本机C验证：本机10000ms时接收到echo=2000ms、pitch=10°/roll=-10°的使能指令，目标被接受。这不是电脑时钟不同步导致的，echo本来就是飞控自身时钟。

若需要防止此前出现过的秒级排队命令执行，应由飞控维护端增加echo年龄检查；停止命令仍应立即回零，不因echo旧而忽略。合适门限要包含5Hz yaw采样间隔、下行、电脑等待与上行，不直接等同单向时延。电脑端已有看门狗不能替代飞控端判断。

源码：[接收解码与更新时间](https://github.com/HuangGL03/XUSTBfly_control/blob/dfe1d83a748459e77c12378f6a98e1499daaf55b/application/mavlink.c#L353)，[目标选择](https://github.com/HuangGL03/XUSTBfly_control/blob/dfe1d83a748459e77c12378f6a98e1499daaf55b/application/Aircraft_ctrl_task.c#L1089)。

### P2：完全断链后，旧目标角可保持约1.5秒

FLOW_PC_TARGET_TIMEOUT_MS=1500。真实函数验证最后一包后1499ms仍输出原10°/-10°目标，1501ms回零。明确的enabled=0包能立即清零，但完全失去通信时不能依赖该包到达。对当前10Hz、已确认低延迟的姿态指令链路，可考虑300～500ms作为地面验证起点，再按实测丢包和抖动设定。这是保护策略建议，不是两端编码不兼容。

源码：[超时参数](https://github.com/HuangGL03/XUSTBfly_control/blob/dfe1d83a748459e77c12378f6a98e1499daaf55b/algorithm/flow_config.h#L30)。

## 已确认一致

- FLOW_RUN_XY_VELOCITY_ENABLE=0，直通姿态分支不再依赖光流XY有效性。
- MAVLink 2 ID248、type=0x8002、payload版本1、小端<BBhhI>；先pitch后roll，0.01°/计数。
- pitch前倾为正、roll左滚为正；与电脑前向误差→正pitch、右向误差→负roll一致，不需要再加Y负号。
- 飞控目标选择对电脑目标逐轴限幅±30°，不是合成倾角限幅。
- enabled=0将两轴目标置零；错误类型/版本不更新目标，尾零截断后正确补零。
- 最新调试字段已改为flow_pitch_set_deg/flow_roll_set_deg和flow_target_*；应查看这些字段与flow_data.control.pitch_target_deg/roll_target_deg。

## 仍启用的功能

FLOW_RUN_HEIGHT_ENABLE=1，仍使用测距定高到1m。舍弃水平速度环不等于关闭定高；如果仅是不相信光流XY速度，可以保留。若整个下视模块都不可靠，应由用户决定是否切为手动油门。

## 验证范围

已静态检查改动和控制路径，并对提取的原始接收/目标选择C函数做本机编译执行，覆盖有符号角度、限幅、停用、错误类型/版本和超时。记录在log/firmware_review/dfe1d83a748459e77c12378f6a98e1499daaf55b/pc_contract_check.log。未进行STM32全工程编译、板上烧录或本轮实飞，不能据此确认硬件动态响应。

# 正常通信、定点/航点与水平轨迹飞行

当前默认输出已改为[位置环直接姿态控制](direct_attitude_control.md)，需要配套飞控角度接口。启动/对齐/轨迹服务沿用，观察改用/fc/attitude_setpoint_sent，角度增益为°/m、每轴限幅±30°；以下速度环说明仅适用于显式horizontal_control_mode:=velocity。

底层仍使用ID254 yaw，不等待ID250。显式选择velocity时的[精简速度接口](compact_mavlink.md)保留；旧ID31/250、实测速度及光流接管说明仅适用于完整遥测模式。

所有终端从工程根目录运行。通信与位置环保持运行，服务和目标请求在另一个终端执行。

当前默认关闭光流里程计接管，控制位置只使用滤波UWB。光流实测速度仍用于原有速度反馈，里程计继续观察和记录；UWB不可用时输出零，持续异常按原有2秒恢复规则清除目标。开关在position_hold.yaml的use_optical_flow_fallback，修改后重启导航；启动参数use_optical_flow_fallback:=true可显式重新启用。

## 启动

```bash
# 终端1：遥控采集、映射、飞控双向通信；默认实际发送
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py
# 终端2：位置环、UWB驱动、场景、RViz及默认实验记录器
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py use_rviz:=true
```

启动后控制器为DISABLED，等待本次对齐。位置入口默认串口为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`、921600、Tag0；可用 `port_name:=... baud_rate:=...` 覆盖。已有UWB驱动时追加 `start_uwb:=false`；不记录时追加 `record_experiment:=false`。不要重复占用串口或UDP9000。

备用链路时，终端1改用下列命令；先停止原通信入口，再启动备用入口：

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication_backup.launch.py
```

此入口只运行飞控桥接，不启动遥控采集/映射，不订阅或发送遥控消息70。本机监听`0.0.0.0:14550`，发送到`10.0.0.1:14555`；Backpack默认接收电脑上行的端口是14555，向电脑发送遥测的目标端口是14550。用户网络截图确认电脑本机为10.0.0.100、Backpack/AP网关为10.0.0.1，发送目标应为后者。姿态31、速度/高度250、心跳及XY速度84接口和发送Y符号适配保持原约定，位置环与下列服务流程相同。地址/端口可通过`remote_ip`、`tx_port`、`rx_port`、`bind_ip`启动参数覆盖。也可在原通信入口中传`use_rc:=false`得到仅桥接进程；`use_joy:=false`单独使用只关闭USB采集，不能代替关闭遥控转发。

速度消息84默认以10Hz发送，降低ELRS空中带宽占用；需要20Hz时在任一通信入口后追加`velocity_rate_hz:=20.0`。默认值在pocket_bench.yaml的`velocity_rate_hz`中修改，重启通信生效。位置环计算仍为50Hz，桥接只取最新指令发送；电脑侧200ms指令有效期、接收消息及轨迹时钟不随发送频率改变。

机头朝map +X、静置后执行：

```bash
bash scripts/ros2.sh ros2 service call /navigation/start_here std_srvs/srv/Trigger '{}'
```

成功后同时记录航向偏置、捕获当时的map XY目标、启用持续位置保持。当前场地+X沿A0→A3。再次调用会重新记录航向并把目标改为新的当前位置，因此每次调用都应朝+X。

服务需要新鲜有效的UWB与ID31姿态；不要求光流在地面有效。尚未收到新鲜ID250时进入WAITING_FEEDBACK，保留目标和航向、输出零；收到后自动继续。飞控重启或姿态参考重置后，重新朝+X调用。

解锁、起飞、降落及遥控/外部速度模式选择由飞控负责，地面服务不执行这些操作。

## 设置目标

```bash
bash scripts/ros2.sh ros2 topic pub --once /navigation/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}'
```

`/goal_pose`也是同一接口，正常RViz使用2D Goal Pose发布到这里。只接受map下有限的XY；忽略目标Z和姿态。每次请求替换当前目标，到达后持续保持，没有自动执行的航点队列。通过 `/navigation/status` 或RViz目标标记确认是否接受。

```bash
# 停用控制器并清除目标；通信和遥控继续
bash scripts/ros2.sh ros2 service call /navigation/enable std_srvs/srv/SetBool '{data: false}'
# 只取消当前目标，输出零，等待后续目标
bash scripts/ros2.sh ros2 service call /navigation/cancel_goal std_srvs/srv/Trigger '{}'
```

## 启动时选择飞行模式

通信入口保持原样。导航入口使用 `flight_mode` 选择模式，默认waypoint：

```bash
# 定点/手动航点：保持start_here当前位置，之后用目标话题或RViz改点
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py flight_mode:=waypoint use_rviz:=true
# 2米直线往返
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py flight_mode:=line use_rviz:=true
# XY水平面的八字
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py flight_mode:=figure8 use_rviz:=true
```

三条命令是三种选择，不能同时运行。可以继续叠加 `use_optical_flow_fallback:=true/false`、`experiment_label:=...`、`start_uwb:=false` 等参数，沿用光流辅助的遮挡对照实验。

### 对齐、起飞准备与开始

选择轨迹模式后，依旧先将机头朝+X并调用 `/navigation/start_here`。此时只定点，记录器等待正式TRACKING，**不会自动开始走轨迹**。准备好且定位、yaw有效时再执行。当前姿态模式不要求光流；旧完整速度模式还需新鲜有效的光流XY，不限制开始时速度低于某个阈值：

```bash
bash scripts/ros2.sh ros2 service call /navigation/start_trajectory std_srvs/srv/Trigger '{}'
```

无需先手动飞到起点上方。轨迹先从当前位置平滑接入起点，再开始直线或八字；接入参考到达起点后，如果飞机实际XY还未进入到达位置容差，就继续把起点作为目标，保持APPROACH阶段，实际到达后才进入TRACKING。起点位置容差由trajectory.yaml的start_position_tolerance_m设置（当前0.15m），与定点到达容差分开，不额外要求起点速度低于某个阈值。控制器仍要求当前位置在基站范围的配置边距内，并通过定位、姿态和航向对齐等检查；地面光流无效时可以正常start_here定点，但不能假装有有效速度来开始轨迹。

提前结束轨迹并保持当时的当前位置：

```bash
bash scripts/ros2.sh ros2 service call /navigation/stop_trajectory std_srvs/srv/Trigger '{}'
```

默认执行1轮：直线从A到B再回A；八字走完闭合一圈回中心，末端减速到零并定点保持。运行中重复start会被拒绝，完成或手动停止后可以再次start。发送手动目标会取消当前轨迹并改为保持该目标；重新start_here也会取消轨迹、重新对齐并定点。Ctrl+C仍自动保存日志。

### 当前基站范围与轨迹

轨迹只规划map X/Y，**不规划高度，不产生Z速度或yaw目标**。高度与原有姿态/航向控制仍由飞控负责。

当前scene.yaml中原始UWB基站XY是A0=(0,0)、A1=(0.078,7.736)、A2=(4.273,7.528)、A3=(4.277,0)。场地约4.3×7.7m。先按uwb.yaml转换到map，再在凸多边形内部保留0.8m边距。现有变换为map X=UWB X、map Y=-UWB Y；新标定沿用该坐标约定时无需改代码或uwb.yaml，+X仍沿A0→A3。

| 轨迹 | 当前map设置 |
|---|---|
| 直线 | Y=-3.868m，X从1.1385m到3.1385m，跨度严格2m |
| 八字 | 中心(2.1385,-3.868)m，X跨度1.6062m，Y跨度3.2124m，两瓣沿Y长边排列 |

当前八字为 `x=cx+Ax*sin(2*phase), y=cy+Ay*sin(phase)`，每瓣长/宽约1，避免把原来的X向双瓣挤成两条细长曲线。`figure8_axis=auto`根据变换后的map范围选择长边；也可指定x或y。`figure8_lobe_aspect_ratio=1.0`限制每瓣的长宽比，包络只会缩小，不向场地边缘扩张。Y仍是水平坐标，不是高度。

移动基站后更新scene.yaml并重启导航和场景显示，会自动重新计算轨迹中心和可用范围，不需要修改控制算法。整个轨迹包络及接入段都检查边距；范围不够容纳配置的直线长度时会拒绝启动规划，不偷偷缩短长度。基站坐标必须与UWB硬件实际输出所用的标定一致；若改变了硬件坐标轴方向、单位或原点，再相应更新uwb.yaml。已有手动航点仍按map绝对坐标解释，不会随基站移动自动平移。

### 时间、速度与位置环

位置参考和解析速度在50Hz位置环内更新，每个map轴分别使用 `v_ref_i + kp_i*(p_ref_i-p_i) + kd_i*(v_ref_i-v_i)`，再经过原来的合速度/加速度限制及机体坐标转换。kp_x/kd_x与kp_y/kd_y可独立调整；它们是世界map轴参数，飞控上的前向/右向速度PID是另一层控制。这里没有积分项。waypoint模式仍使用定点P/PD。

接入段为起止速度/加速度均为零的五次曲线；轨迹相位有加减速段，起止速度为零。默认直线相位周期27秒、八字80秒，另有5秒起止相位缓冲及按距离自动计算的接入时间。当前八字参考速度上界约0.179m/s，直线约0.233m/s；配置的统一参考限速为0.25m/s、限加速度为0.2m/s²。周期过短时会自动放慢，不能通过缩短周期绕过参考速度/加速度限制。

接入曲线到达起点后，仍等待实际XY进入起点容差，避免飞机还没到就开始跑轨迹。`start_position_tolerance_m=0.15`单独控制这个交接，不再强求5cm定位；航点到达的5cm容差保持独立，没有添加悬停速度门限。这会减少过严位置判据造成的等待，但不能替代速度环的跟踪能力。

短时UWB异常、有效光流接管期间轨迹继续执行；轨迹运行时要求真实速度有效，不使用贴地NaN按零的定点假设。位置、实测速度或姿态等关键输入无效时，输出零并暂停轨迹运行时钟，暂停期间解析参考速度也记为零。窗口内恢复后继续原轨迹，指令从零限加速恢复。持续故障、飞控复位、取消目标或停用控制器会取消轨迹，不会在故障解除后自动重启。

### 参数与显示

[trajectory.yaml](../src/flapping_uav_bringup/config/trajectory.yaml)集中配置2m直线长度、八字方向/形状/面积、周期、轮数、边距、起点容差及参考运动限制。figure8_fit_scale当前为0.6，更小值会缩小八字，但不改变中心和直线长度。参考速度还会受到控制器上限约束。开始速度阈值已取消，更改配置后重启导航。

启动参数 `trajectory_cycles:=2`可改轮数，`trajectory_period_s:=80`可覆盖该模式周期，0表示使用配置。八字的面积是水平XY范围；更改fit_scale不会更改高度。

RViz中的Planned Horizontal Trajectory显示完整水平曲线，另以青色显示接入段。`/navigation/status`增加flight_mode、trajectory_phase、trajectory_running、trajectory_time_s、实际相位周期及解析map速度。phase包括WAITING_START、APPROACH、TRACKING、FINISHED、CANCELLED。

论文日志reference.csv连续保存执行过的期望位置与解析速度；不会把每个轨迹样本当成一个离散航点事件。轨迹图直接叠加期望曲线与实际定位，不绘制未执行的未来轨迹。比较两组遮挡实验时应选择相同flight_mode、轨迹参数和任务，并结合phase/运行时间区分准备和飞行段。

## 查看数据

```bash
bash scripts/ros2.sh ros2 topic echo /navigation/status
# 内部前正/右正，与当前发送轴向一致
bash scripts/ros2.sh ros2 topic echo /fc/velocity_setpoint_body
# 已适配符号并成功交给UDP的消息84数值，不是飞控执行回执
bash scripts/ros2.sh ros2 topic echo /fc/velocity_setpoint_sent
```

默认位置控制使用 `/localization/position` 的可靠UWB/光流接管结果，以及 `/fc/attitude`、`/fc/body_motion`；关闭辅助时保留直接UWB流程，按map系P/PD解算并在内部旋转为水平前/右速度。当前桥接打包保持X前正、Y右正，velocity_y_sign=1；已移除旧Y取反。原始UWB Y先在UWB节点转换成map Y，不要在控制器中重复取反。

## 状态与异常恢复

| state | 含义 |
|---|---|
| DISABLED | 未启用，或通过enable服务停用 |
| WAITING_FEEDBACK | 已记录航向和目标，等待新鲜ID250 |
| IDLE / READY | 等待目标，或已有目标但未启用 |
| ACTIVE | 正在计算XY位置控制 |
| REACHED | 位置和实测速度满足到达条件；仍持续保持 |
| INPUT_HOLD | 短暂输入异常，输出零并保留原目标 |
| FAULT | 故障已锁存，目标清除，需重新start_here |
| CANCELLED | 当前目标取消，输出零 |

关键输入短暂异常后输出零，从首次异常检测起保留原目标2秒；窗口内恢复则从零重新计算并继续原目标。持续超时或确认飞控参考重置后清除目标，不自动恢复。`Inputs recovered; previous fault remains latched`表示输入恢复了，但旧故障已清除目标，需重新朝+X调用start_here。

当前心跳只作诊断；单独丢心跳不停止仍有新鲜姿态和速度消息的控制。姿态与速度源ID、坐标和时间仍要通过检查，冻结设备时间不能刷新有效期。

当前 `invalid_velocity_as_zero=true`：在定点/航点保持时，贴地光流XY无效但ID250消息新鲜，控制器把XY反馈暂按零使用，原始话题保留NaN和false，状态标明 `feedback_assumed_zero`。这项设置不自动判断是否离地；轨迹执行期间禁用此假设。消息缺失、冻结或过期不按零测量处理，而是进入恢复等待。

## 参数与默认值

实际配置在 [position_hold.yaml](../src/flapping_uav_bringup/config/position_hold.yaml)：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| kp_x / kd_x | 1.5 / 0.0 | map X的位置比例与速度阻尼，无积分项 |
| kp_y / kd_y | 1.5 / 0.0 | map Y的位置比例与速度阻尼，无积分项 |
| max_speed_mps | 2.0 | XY速度向量限制 |
| max_acceleration_mps2 | 1.0 | 速度向量变化限制 |
| arrival_position_tolerance_m | 0.05 | 到达位置阈值 |
| arrival_speed_tolerance_mps | 0.05 | 到达实测速度阈值 |
| arrival_dwell_s | 0.5 | 连续满足到达条件时间 |
| position_timeout_ms | 500 | UWB时效 |
| attitude_timeout_ms / feedback_timeout_ms | 1500 / 1500 | 姿态与ID250时效 |
| heartbeat_required | false | 心跳仅作诊断 |
| input_recovery_grace_ms | 2000 | 输入恢复窗口 |

这些是现有实验配置，尚未完成实飞整定。旧配置只提供kp_xy/kd_xy时仍可使用，未指定的轴继承相应公共值；显式轴参数优先，当前文件直接使用四个轴参数。修改后重启位置环，再重新朝+X记录；改限速时通信也需重启，因为它会加载同一速度上限。发送端的200ms命令超时、1500ms姿态时效和 `velocity_y_sign=1` 位于同一文件的mavlink_bridge_node部分。

2026-11-25 04:01:56日志显示：进入轨迹起点约44.2秒，旧参考峰值约0.592m/s，跟踪期实际光流速度中位数约0.263m/s，跟踪误差中位数约1.40m，电脑合速度指令中位数约2.00m/s，已接近限幅。盲目增大位置P不能提高已饱和的输出，还可能加重接近段摆动。新配置先降低参考运动速度并保留该次P/D值；先验证飞控实际采用的速度目标、速度环跟踪与遥控回中，再分别整定机体前/右速度环及map位置P/PD。不能把离线几何/仿真通过当作新参数已实飞验证。

通信地址、发送开关可通过 [communication.launch.py](../src/flapping_uav_bringup/launch/communication.launch.py) 参数覆盖。遥控映射在pocket_bench.yaml，UWB变换在uwb.yaml，基站显示在scene.yaml。

位置入口默认启动自动记录器；定点/航点在start_here成功后开录，直线/八字到达起点进入TRACKING后才开录，Ctrl+C结束导航时自动保存原有三组论文图、同次原始UWB/补偿定位两张对照图和原始CSV，不录制rosbag。操作见 [实验日志](experiment_logging.md)。当前默认关闭[光流接管](fusion_odometry.md)，控制使用滤波UWB；恢复完整光流遥测并显式启用后才使用里程计接管。

## 常见启动问题

- `waiting for service to become available...`：检查位置控制器是否启动成功。通信、位置环和服务调用应使用不同终端，不要先Ctrl+C停止前两个入口。
- YAML中小数参数建议写为 `2.0`，当前日常launch也会把合法数值规范成double；无效类型会直接报错。
- 串口Permission denied：检查设备组和当前会话权限；串口被占用时先关闭NAssistant或重复驱动。
- RViz没有飞机：查看 `/fc/attitude` 与 `/navigation/heading_alignment`；没有新鲜有效姿态时不会保留旧模型。仅重开窗口见 [RViz说明](rviz_scene.md)。

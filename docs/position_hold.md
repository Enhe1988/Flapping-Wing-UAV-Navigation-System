# 正常通信、位置保持与单目标点控制

所有终端从工程根目录运行。通信与位置环保持运行，服务和目标请求在另一个终端执行。

## 启动

```bash
# 终端1：遥控采集、映射、飞控双向通信；默认实际发送
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py
# 终端2：位置环、UWB驱动、场景、RViz及默认实验记录器
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py use_rviz:=true
```

启动后控制器为DISABLED，等待本次对齐。位置入口默认串口为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`、921600、Tag0；可用 `port_name:=... baud_rate:=...` 覆盖。已有UWB驱动时追加 `start_uwb:=false`；不记录时追加 `record_experiment:=false`。不要重复占用串口或UDP9000。

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

## 查看数据

```bash
bash scripts/ros2.sh ros2 topic echo /navigation/status
# 内部前正/右正，未做输出Y符号适配
bash scripts/ros2.sh ros2 topic echo /fc/velocity_setpoint_body
# 已适配符号并成功交给UDP的消息84数值，不是飞控执行回执
bash scripts/ros2.sh ros2 topic echo /fc/velocity_setpoint_sent
```

位置控制器使用 `/uwb/position`、`/uwb/status`、`/fc/attitude`、`/fc/body_motion`，按map系P/PD解算并在内部旋转为水平前/右速度。桥接只在打包时把Y取反，X不变。原始UWB Y先在UWB节点转换成map Y，不要在控制器中重复取反。

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

当前 `invalid_velocity_as_zero=true`：贴地光流XY无效但ID250消息新鲜时，控制器把XY反馈暂按零使用，原始话题保留NaN和false，状态标明 `feedback_assumed_zero`。这项设置不自动判断是否离地。消息缺失、冻结或过期不按零测量处理，而是进入恢复等待。

## 参数与默认值

实际配置在 [position_hold.yaml](../src/flapping_uav_bringup/config/position_hold.yaml)：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| kp_xy / kd_xy | 1.0 / 0.0 | 位置比例与速度阻尼，无积分项 |
| max_speed_mps | 2.0 | XY速度向量限制 |
| max_acceleration_mps2 | 1.0 | 速度向量变化限制 |
| arrival_position_tolerance_m | 0.05 | 到达位置阈值 |
| arrival_speed_tolerance_mps | 0.05 | 到达实测速度阈值 |
| arrival_dwell_s | 0.5 | 连续满足到达条件时间 |
| position_timeout_ms | 500 | UWB时效 |
| attitude_timeout_ms / feedback_timeout_ms | 1500 / 1500 | 姿态与ID250时效 |
| heartbeat_required | false | 心跳仅作诊断 |
| input_recovery_grace_ms | 2000 | 输入恢复窗口 |

这些是现有实验配置，尚未完成实飞整定。修改后重启位置环，再重新朝+X记录；改限速时通信也需重启，因为它会加载同一速度上限。发送端的200ms命令超时、1500ms姿态时效和 `velocity_y_sign=-1` 位于同一文件的mavlink_bridge_node部分。

通信地址、发送开关可通过 [communication.launch.py](../src/flapping_uav_bringup/launch/communication.launch.py) 参数覆盖。遥控映射在pocket_bench.yaml，UWB变换在uwb.yaml，基站显示在scene.yaml。

位置入口默认记录实验，操作见 [实验日志](experiment_logging.md)。独立 [融合观察器](fusion_odometry.md) 不参与当前控制；UWB异常时位置环仍按上述规则处理，不会自动使用融合结果。

## 常见启动问题

- `waiting for service to become available...`：检查位置控制器是否启动成功。通信、位置环和服务调用应使用不同终端，不要先Ctrl+C停止前两个入口。
- YAML中小数参数建议写为 `2.0`，当前日常launch也会把合法数值规范成double；无效类型会直接报错。
- 串口Permission denied：检查设备组和当前会话权限；串口被占用时先关闭NAssistant或重复驱动。
- RViz没有飞机：查看 `/fc/attitude` 与 `/navigation/heading_alignment`；没有新鲜有效姿态时不会保留旧模型。仅重开窗口见 [RViz说明](rviz_scene.md)。

# 直接发送机体速度的链路测试

本页用于历史速度模式和链路排查。当前默认已经是[目标角控制](direct_attitude_control.md)，对应测试程序为attitude_test.py。仅在飞控匹配0x8001速度格式、通信显式选择horizontal_control_mode:=velocity时使用下面的velocity_test.py；不能把速度测试值发送给0x8002角度固件。精简速度模式仍需ID254 yaw用于真实时间回显。下文ID84描述仅适用于velocity_protocol:=legacy的旧固件。

测试绕过UWB、里程计、航向对齐和位置环，直接向现有通信桥接发布机体水平速度。X向前为正，Y向右为正，单位m/s；Z始终为0。旧legacy模式保持ID84帧8/XY掩码，compact速度模式用ID248 0x8001；两者均保留发送Y符号和心跳，不修改正常导航参数。没有位置环的加速度限制，输入即阶跃；默认每条非零指令持续3秒后回零。

先停止position_hold.launch.py及其他速度发布者，保留一个通信入口。备用链路的启动方法：

```bash
cd ~/uwb
# 终端1；如果备用通信已经运行，不要重复启动
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication_backup.launch.py horizontal_control_mode:=velocity
# 终端2：交互式速度测试
bash scripts/ros2.sh ros2 run flapping_uav_gcs velocity_test.py
```

在终端2逐行输入：

```text
0.3 0 3
0 -0.3 3
0.2 0.2 5
stop
q
```

分别为前向0.3m/s持续3秒、左向0.3m/s持续3秒、前右各0.2m/s持续5秒、立即回零、退出。每条输入立即替换上一条，未填时间采用默认3秒。默认速度矢量上限1m/s，NaN/Inf或超限输入被拒绝。需要调整默认时间或上限时追加`--duration 5 --max-speed 0.5`；上限还受通信桥接的max_body_speed_mps约束。

也可以单独发一个阶跃并自动结束：

```bash
bash scripts/ros2.sh ros2 run flapping_uav_gcs velocity_test.py --command 0.3 0 --duration 3
```

Ctrl+C或q会持续约0.35秒发布零后退出，通信节点原有200毫秒指令超时归零保留。程序检测到其他速度发布者会退出，避免两路控制同时发布。测试不需要姿态/光流有效：输入已经是机体水平速度，不进行姿态旋转，测试消息的attitude_age_ms明确置0。正常导航的姿态和定位检查保持原状。

ROS发布为50Hz，通信节点默认仍以10Hz打包ID84。对齐到下一帧发送通常需要0～100毫秒，终端会打印输入时间及首个匹配UDP回报的时间差。该差值还包含ROS调度和回报传输，**仅代表电脑端，不是飞控收到指令的延迟**。

在另一终端观察实际交给UDP的速度：

```bash
bash scripts/ros2.sh ros2 topic echo /fc/velocity_setpoint_sent
```

飞控debug分别观察接收值`pc_data.vx_set_mps/vy_set_mps`、`velocity_rx_count`和`velocity_update_ms`，再观察实际采用值`aircraft_control.flow_data.control.vx_set_mps/vy_set_mps`。收到与采用是两步：按之前检查的固件，光流控速为B中档、C初始档，右摇杆回中才采用电脑速度。若当前固件已更新，以实际仲裁代码为准。

每次测试自动保存`log/velocity_tests/<时间>_velocity_test.csv`：

| 字段 | 含义 |
|---|---|
| wall_time_utc / elapsed_s | 电脑UTC时间 / 测试开始后的单调秒数 |
| event | input手动输入或自动回零；publish发给通信节点；udp_sent实际UDP发送监测；fc_motion收到ID250实测速度 |
| step | 本地当前阶跃编号；不写入MAVLink，不是飞控确认编号 |
| vx_mps / vy_mps | 对应该事件的前向/右向速度 |
| valid | 对应该事件的有效标志 |
| since_input_ms | 本地当前输入后的时间；接收事件包含传输/回调调度时间 |
| detail | 原因和说明 |

ID250是实测速度，不是ID84接收确认；电脑和飞控时钟未同步，不能直接相减得到单向链路延迟。测试的明显阶跃方便与飞控debug曲线对应，精确量化需飞控回传接收时间/目标值或使用共同时间基准。没有定位限位，请自行控制测试空间和持续时间。

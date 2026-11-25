# UWB位置环直接输出姿态目标

本模式在电脑上根据位置误差计算俯仰、滚转目标，不经过光流水平速度环。UWB原始检查和滤波、+X航向对齐、定点/航点/水平轨迹、断流恢复和自动日志沿用。飞控继续负责姿态/角速度内环、遥控模式和推力。

## 当前飞控接口核对

已核对用户维护的[飞控dfe1d83](https://github.com/HuangGL03/XUSTBfly_control/commit/dfe1d83a748459e77c12378f6a98e1499daaf55b)，0x8002版本1、pitch/roll顺序、0.01°单位、方向及电脑目标角±30°限幅均与本机一致，不需要旧74d6a5c配套补丁。飞控由用户维护，本工程只修改电脑端；历史补丁不再作为当前安装步骤。

**当前飞控编译配置的MAVLINK_QGC_FLOW_DEBUG_ENABLE=1会停止发送ID254 yaw，不能直接用于ROS导航。** 应由飞控维护端改为0并重新编译烧录，或另行保证yaw回传不中断。FLOW_RUN_XY_VELOCITY_ENABLE保持0，表示直接姿态控制。

最新飞控接收字段为pc_data.flow_pitch_set_deg、flow_roll_set_deg、flow_target_echo_ms、flow_target_update_ms和flow_target_rx_count；实际送入姿态环的目标可查看aircraft_control.flow_data.control.pitch_target_deg/roll_target_deg。

当前实现的指令接收超时为1500ms，echo仅记录，未校验其总年龄。电脑的200ms发布看门狗不等同于飞控断链保护：飞控完全收不到新包时，仍可能维持旧目标角约1.5秒。是否收紧至300～500ms及按回显时刻拒绝旧命令，由飞控维护端结合实际抖动调整。对应审查见[最新飞控联调审查](firmware_review_dfe1d83.md)。

B中档/C初始档进入原FLOW_RUN；C中档且右杆偏离中心时允许手动接管，沿用最新飞控RUN映射，回中后交回电脑。B第三档/C初始档退出到手动RUN。直接姿态控制分支不再以光流XY有效为前提。

高度仍由原Flow_Run_Thrust处理。当前FLOW_RUN_HEIGHT_ENABLE=1，测距失效时按原逻辑过渡到手动油门；若要完全不用测距定高，由飞控维护端设置FLOW_RUN_HEIGHT_ENABLE=0。电脑不发送推力或期望yaw。

## 角度计算与参数

```text
map位置误差 → X/Y独立PID角度需求 → 根据当前yaw旋转到前/右方向
           → pitch=前向角度需求，roll=负的右向角度需求
           → 每轴限幅±30° → 每轴目标变化率限制 → 飞控姿态环
```

当前飞控姿态约定：前倾pitch为正，左滚roll为正。因此机头对准map +X时，目标在前方给正pitch，目标在右方给负roll；yaw变化时先做坐标旋转，不固定把世界X/Y当作俯仰/滚转轴。

在[src/flapping_uav_bringup/config/position_hold.yaml](../src/flapping_uav_bringup/config/position_hold.yaml)修改，重启导航生效。以下为初始调试示例，实际运行值以本机配置及日志参数快照为准：

```yaml
horizontal_control_mode: attitude
attitude_kp_x_deg_per_m: 10.0
attitude_kp_y_deg_per_m: 10.0
attitude_ki_x_deg_per_m_s: 0.5
attitude_ki_y_deg_per_m_s: 0.5
attitude_kd_x_deg_s_per_m: 3.0
attitude_kd_y_deg_s_per_m: 3.0
attitude_integral_limit_deg: 2.0
attitude_integral_zone_m: 0.75
attitude_integral_max_speed_mps: 0.4
attitude_derivative_limit_deg: 8.0
attitude_velocity_filter_tau_s: 0.2
attitude_velocity_limit_mps: 3.0
attitude_sample_max_gap_s: 0.25
attitude_max_angle_deg: 30.0
attitude_max_rate_deg_s: 60.0
```

P沿用本机最近的10°/m，对准+X、前方1m误差时，P项为+10°pitch。I初值0.5°/(m·s)，D初值3°/(m/s)，X/Y分别调节。这些是待真机验证的初始值，不是已经整定好的参数。旧kp_x/kp_y、max_speed_mps、max_acceleration_mps2不决定本模式的角度输出，轨迹自身的速度/加速度参数仍决定参考点如何移动。轨迹的m/s前馈不会直接加到角度中。

两轴分别限幅，允许pitch=30°且roll=-30°同时出现；不是合成倾角30°限制。电脑桥接和飞控补丁各自还会限制每轴±30°，不允许通过改配置提高到30°以上。60°/s是目标角变化率，改变目标后会立即开始变化，到30°至少约0.5秒；它不是实测角速度反馈。D使用滤波UWB的位置差分估计运动速度，不使用光流，也不把该估计当作飞控实际速度。

每个map轴的控制律为 `u = Kp*(目标位置-当前位置) + I - Kd*UWB估计速度`。D对实际位置求导，不对位置误差求导，设置新航点不会产生微分冲击；轨迹也用同一制动律，尚未增加参考速度前馈。位置差分使用观测时间戳，重复读取同一定位点不会生成虚假的零速度。速度经0.2s一阶低通，差分速度向量先限3m/s，D角度向量最多8°。超过0.25s没有新观测时停止采用旧D，间隔过大后的首点重新初始化；原有500ms定位超时保护继续生效。

I只在位置误差距离≤0.75m、UWB估计速度≤0.4m/s且收到新有效位置时积分。积分角度向量长度受attitude_integral_limit_deg限制，旋转后的任一机体轴积分补偿也不会超过该上限。对总角度限幅和角度变化率限制都做条件积分，避免继续向饱和方向累积。新离散航点、start_here、取消、禁用、输入异常和飞控重启会清积分；连续轨迹参考更新保留积分。到达位置容差、UWB估计速度低于arrival_speed_tolerance_mps并驻留足够时间后标记REACHED，**仍保持PID补偿，不强制目标角归零**，避免丢掉抗恒定扰动所需的配平角。

目前回传只有yaw，电脑不知道飞控何时遥控接管，无法自动冻结该时段积分。I区间/速度门限与积分限幅只能限制影响；需频繁手动干预时可先把两个Ki设为0。I/D设为0即可关闭对应项，但PID模式到达状态始终保留连续控制。

UWB/yaw异常时立即发禁用零角度并清I/D状态；2秒内输入恢复继续原目标，持续失效或飞控重启仍清除目标。恢复从零角度重新按变化率增加。

## 紧凑报文

飞控→电脑仍为ID254 ind=1 yaw(rad)，5Hz；双向心跳各1Hz，网络端口不变。

电脑→飞控为MAVLink 2的ID248，message_type=0x8002，目标网络0、系统/组件1/1，10Hz。payload尾部补零，前10字节按小端`<BBhhI`：

| 字节 | 含义 |
|---|---|
| 0 | 版本1 |
| 1 | enabled，0停止/回水平，1采用目标角 |
| 2–3 | pitch，int16，0.01°/计数，前倾正 |
| 4–5 | roll，int16，0.01°/计数，左滚正 |
| 6–9 | 最近有效yaw的time_boot_ms原样回显 |

例如pitch=5°、roll=-12°编码为500、-1200。完整帧最多27字节，保留MAVLink 2尾零裁剪。该格式与旧速度格式长度相同，但类型和物理单位不同，不能混用。

最新飞控dfe1d83的接收看门狗为1500ms；yaw时间回显仅记录，不代表已实现旧命令拒绝。若后续增加时间检查，该年龄包含yaw采样、下行、电脑等待和上行，不是单向时延。电脑仍保留200ms发布看门狗。

## 使用命令

先退出旧通信、导航和速度测试；飞控关闭互斥的QGC调试回传并烧录匹配代码后，重新启动：

```bash
cd ~/uwb
# 终端1：备用通信，默认已选择角度协议
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication_backup.launch.py
# 终端2：仅角度测试，不能与导航同时运行
bash scripts/ros2.sh ros2 run flapping_uav_gcs attitude_test.py
# 测试程序输入：pitch_deg roll_deg 秒数，例如 5 0 3
# stop回水平；q/Ctrl+C发零角度并退出
```

先确认飞控debug字段pc_data.flow_pitch_set_deg/flow_roll_set_deg与实际控制pitch_target_deg/roll_target_deg匹配，再退出角度测试、启动正常导航：

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py use_rviz:=true
# 或追加flight_mode:=line / figure8
# 机头朝map +X后在另一终端执行
bash scripts/ros2.sh ros2 service call /navigation/start_here std_srvs/srv/Trigger '{}'
# 轨迹模式对齐后仍需手动开始
bash scripts/ros2.sh ros2 service call /navigation/start_trajectory std_srvs/srv/Trigger '{}'
```

观察话题：

```bash
bash scripts/ros2.sh ros2 topic echo /fc/attitude_setpoint_body
bash scripts/ros2.sh ros2 topic echo /fc/attitude_setpoint_sent
bash scripts/ros2.sh ros2 topic echo /navigation/status
bash scripts/ros2.sh ros2 topic echo /navigation/attitude_pid
```

两个角度话题字段明确为pitch_deg、roll_deg；sent表示成功交给电脑UDP，不是飞控执行确认。原/fc/velocity_setpoint_body不会再由导航发布角度，NavigationStatus.velocity_map在本模式为NaN。直接调试请使用attitude_test.py，不再用velocity_test.py。

正常position_hold入口中的RViz飞机模型使用`/uwb/position_filtered`，与当前UWB控制输入一致；原来模型直接使用原始UWB，视觉跳动会比控制输入大。`Raw UWB Tag XY and Validity`和`Raw UWB XY History`仍显示原始点/轨迹，想专注观察滤波模型时可取消这两项勾选。yaw仍直接取飞控回传，当前只有航向、没有实际俯仰/滚转；模型保持水平不代表真机没有倾斜。

控制位置滤波在`config/fusion_odometry.yaml`的`uwb_filter_tau_s`，当前仍为0.10秒。增大时间常数会更平滑，同时增加位置反馈滞后。单独启动uwb.launch.py或scene.launch.py时默认模型仍用原始UWB，便于不启动滤波节点也能检查定位；滤波节点已运行时可追加`scene_position_topic:=/uwb/position_filtered`。正常导航也可通过该参数切换模型的数据源，参数只影响显示，不改变控制和CSV原始记录。

日志仍在log/experiments/，增加attitude_desired.csv和attitude_sent.csv，自动生成report/attitude_targets.png/pdf。新增attitude_pid.csv和report/attitude_pid.png/pdf，保存/展示map X/Y的P/I/D角度分项、UWB估计速度、估计有效性、积分更新与限幅状态。分项尚未旋转到机体系，也未经过总角度限幅及变化率限制，不能直接当作已发送pitch/roll；实际目标仍看attitude_sent.csv。原始位置、参考位置和yaw继续记录。未回传的实际roll/pitch、角速度、速度不补零，也不把期望角度画成期望速度。直接测试记录在log/attitude_tests/。

回归旧速度模式时，通信与导航都需显式追加horizontal_control_mode:=velocity，并配合旧速度飞控固件；不进行自动协议猜测。

```bash
# 私有ROS域、本机UDP、合成UWB，无硬件访问
bash scripts/ros2.sh python3 scripts/check_compact_communication.py --attitude
```

轨迹模式的日志从实际到达起点、进入TRACKING时才开录，不再包含首次APPROACH；定点/航点仍在start_here成功后开录。开关见experiment_logging.yaml的start_on_trajectory_tracking。Ctrl+C自动保存及原始/滤波UWB两图保持不变。

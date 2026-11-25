# 论文用轻量实验日志

正常飞行只需启动通信与位置环。定点/航点模式在航向对齐后自动开录；直线/八字模式在实际到达轨迹起点、进入TRACKING后才自动开录，跳过起飞准备和APPROACH。按Ctrl+C关闭导航入口时，自动落盘CSV并生成图表；不需要手动开启、停止或保存日志。**新实验不录制rosbag，不保存全量MAVLink、遥控、诊断或二进制话题流。**

当前默认是位置到角度PID模式：期望/UDP发送角度保存在`attitude_desired.csv`、`attitude_sent.csv`，图为`report/attitude_targets.png/pdf`。`attitude_pid.csv`新增map X/Y的P/I/D角度分项（deg）、由UWB差分低通得到的速度估计（m/s）、valid、velocity_estimate_valid、integrating和limited；图为`report/attitude_pid.png/pdf`。分项在旋转与总限幅前，limited包含角度及变化率限幅。此估计不是光流反馈，未回传的实际roll/pitch、角速度及速度继续留NaN/空表；以下速度图说明对应旧速度模式。

## 正常使用

```bash
# 终端1：通信，保持运行
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py
# 终端2：导航、UWB、显示及等待对齐的记录器
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py use_rviz:=true
# 终端3：机头朝map +X，记录航向并启用当前位置保持
bash scripts/ros2.sh ros2 service call /navigation/start_here std_srvs/srv/Trigger '{}'
```

记录器先等待start_here成功发布本次有效航向。waypoint模式此时创建目录；line/figure8默认继续等待/navigation/status的有效TRACKING状态，此前不创建实验目录、不缓存或回放准备/接入过程。到达起点的判定由导航节点负责，记录器不另设一套到点距离。旧的过期对齐、无效TRACKING、旧状态消息都不能启动记录。正常启动导航入口已包含记录器，不要再启动第二个。

设置目标和飞行操作沿用原流程。实验结束，在**位置环所在终端按Ctrl+C**，等待 `Paper experiment saved: ...` 后关闭终端。导航进程单独退出时，启动文件也会停止其余导航子进程并让记录器收尾。通信入口仍独立运行。

每次重新启动导航并满足开录条件后创建新目录。一次记录期间再次调用start_here，继续写入当前实验并记录新的航向和目标，不覆盖已有数据。开录后持续记录至Ctrl+C，包含结束后的保持/手动降落时间；不自动识别空中/落地，也不自动在FINISHED时结束。

开关在experiment_logging.yaml的start_on_trajectory_tracking，默认true；设false可恢复所有模式对齐后开录。flight_mode由正常导航入口传给记录器。/experiment/status在对齐后、到达轨迹起点前显示waiting_for_trajectory；若在开录前关闭导航，不产生空实验文件夹。

## 核心数据与图

| 论文内容 | 未滤波原始CSV | 图表 |
|---|---|---|
| 期望位置/实际轨迹 | trajectory.csv、control_position.csv、reference.csv | trajectory.png / trajectory.pdf |
| 同次UWB滤波前/滤波后 | trajectory.csv、uwb_filtered.csv、reference.csv | trajectory_uwb_raw.png/pdf、trajectory_uwb_filtered.png/pdf |
| 期望速度/实际速度 | velocity_desired.csv、velocity_actual.csv | velocity.png / velocity.pdf |
| 三轴角度/角速度 | attitude.csv | attitude.png / attitude.pdf |
| 同次实验的原始UWB与补偿定位对照 | localization_comparison/下的原始点、期望参考及定位状态CSV | 两张独立轨迹图，各有PNG/PDF |

PNG为300dpi，PDF为矢量图。report/保留原有轨迹、角度/速度等图，同时分别增加UWB滤波前和滤波后两张图；localization_comparison/额外保存同次原始UWB与光流定位选择输出对照。HTML预览页引用生成的PNG，不内嵌重复图片。不保存全量话题CSV或二进制bag。

### UWB滤波前后两张独立轨迹图

`report/trajectory_uwb_raw.png/pdf`只使用trajectory.csv的原始UWB；`report/trajectory_uwb_filtered.png/pdf`只使用uwb_filtered.csv中本次实验实际记录的在线低通位置。每图包含XY轨迹及X/Y随时间曲线，并显示相同的期望参考/离散目标，坐标与时间范围完全一致。两个CSV保留各自原始数值和时间，不额外复制一套点，也不为绘图再次滤波、插值或补零。

这是UWB自身在线滤波效果的对照，与下文的“光流里程计补偿”是不同的比较，不能用两图证明光流接管发生。旧日志若未记录uwb_filtered.csv则不伪造滤波后图，报告会提示缺少数据。未来实验结束自动生成两图；已有完整日志重新export即可补生成。

### 同次遮挡实验的定位对照

正常启用光流辅助飞行，满足上文开录条件后自动记录，在UWB被干扰/遮挡的过程中继续保留原始UWB和在线补偿估计。Ctrl+C结束导航后，无需额外命令，自动生成：

```text
localization_comparison/
├── uwb_raw_trajectory.png / .pdf          # 同次原始UWB定位
├── flow_compensated_trajectory.png / .pdf # 同次在线UWB/光流位置选择输出
├── uwb_raw_points.csv                    # trajectory.csv的逐字节副本
├── flow_compensated_points.csv           # 选择器XY原始点、时间、有效性和来源
├── reference_points.csv                  # reference.csv的逐字节副本
├── source_status.csv                     # position_sources.csv的逐字节副本
└── comparison.json                       # 坐标范围、接管区间、样本数与数据来源
```

两张图均包括XY轨迹和X/Y时间曲线，使用完全相同的坐标范围、时间范围与期望轨迹。补偿图按UWB、FLOW_ODOMETRY、UWB_BLEND标色；时间图中红色淡背景表示算法未接受UWB的时段，不等同于人工遮挡动作的精确起止时刻。

原始UWB跳变点按记录保留，不因对照而删除或裁切。断流超过配置中的UWB时效时断线，不生成虚拟位置。补偿输出无效时也断线；CSV仍保留无效原始数值及valid=0，便于后处理。若实验未记录有效光流接管，报告会明确提示，不将两条相同曲线解释为已发生补偿改善。

flow_compensated_points.csv里的x_m/y_m来自position_sources.csv的selected_x_m/selected_y_m，保持原来的数值字符串和采样时刻；其余字段包含valid、source、uwb_healthy、flow_valid、数据年龄、innovation_m、同步次数和光流里程计XY。正常时选择可靠UWB，异常时使用光流接管，恢复后过渡切回。这是在线定位选择器输出，具体是否用于本次控制以control_position.csv和comparison.json的assistance_used_by_controller为准。

两路采样频率不同，不插值成一对一测量；source_status.csv中的raw_uwb坐标会重复保留最后一次收到的位置，单独处理原始UWB点应使用uwb_raw_points.csv。原始UWB是驱动已经发布的map位置，被驱动拒绝而未发布的点无法从此日志还原。原始点未经离线滤波，后续滤波结果应另存文件，并在图注注明方法和参数，保留原始CSV供复核。

此对照可展示同一飞行中的定位稳定性与连续性。它不代表两个独立的“无辅助/有辅助”飞行架次，也不是独立物理真值；若需要控制性能的开关辅助对照，继续使用下文的compare-positions。

### 轨迹

- `trajectory.csv`逐条保存收到的原始UWB map XYZ，其中Z为未验证高度，不进入水平控制。
- `uwb_filtered.csv`保存质量门限通过后的在线低通XY，时间戳与原始观测一致，Z保留原值。在线滤波不覆盖原始点。
- `control_position.csv`保存位置环实际使用的位置、有效性、UWB/光流/过渡来源与辅助开关。轨迹图分别标出原始UWB与实际使用的定位估计，不把它们当作独立真值。
- `position_sources.csv`保存光流里程计、原始UWB、候选控制位置、切换/质量及时效字段，用于研究遮挡过程；不离线改写这些数值。
- `reference.csv`逐条保存导航节点接受的目标XY、是否有目标、启用/输入有效状态、控制状态、期望map XY速度，以及飞行模式、轨迹阶段/运行时间/解析速度。
- 两路保留各自原始采样时刻，不强行插值或同步为相同频率。
- waypoint模式保存分段目标；line/figure8模式连续保存实际执行的平滑XY参考、解析map速度、运行时钟和阶段。XY图按模式显示离散目标或执行过的期望曲线；不把尚未执行的计划当作已执行参考。

### 速度

- `velocity_desired.csv`为位置环计算的水平机体系期望速度，X前正、Y右正，单位m/s。
- `velocity_sent.csv`保存桥接成功交给本机UDP socket的ID84打包值：forward_wire_mps/lateral_wire_mps/z_wire_mps、输入有效标志、指令/姿态年龄与原因。lateral_wire_mps已应用发送符号参数，当前velocity_y_sign=1、向右为正；旧实验可能为-1，以该次参数快照为准；这是本机发送记录，不是飞控接收或执行回执。只接收模式下此表为空，不能用内部期望速度推测已发送。
- `velocity_actual.csv`为ID250回传的同系前向/右向速度，另外保留高度与各自有效标志。
- 速度图比较内部期望与回传的同系前正/右正数据。当前发送也保持该方向；历史实验的发送符号适配不用于这组跟踪对比。
- 无效NaN和原始有效标志保留，不把贴地无效光流伪装成测量零；图中无效数据留空。

### 姿态与角速度

`attitude.csv`同时保存：

- 接收时间、消息源时间、设备时间及重置计数；
- 回传四元数qw/qx/qy/qz；
- 未做时间滤波的roll、pitch、飞控参考yaw和对齐后的map yaw，单位deg；
- ID31回传的机体三轴角速度p/q/r，单位rad/s，以及独立有效标志。

图中三轴角度使用roll、pitch与map yaw，角速度直接使用p/q/r。p/q/r是机体轴角速度，不是欧拉角数值微分，也不应直接混称为三个欧拉角的导数。桥接已补齐ID31这些字段；若固件未填充它们，日志只能保存固件实际发送的值，不能还原未发送的测量。

CSV记录不额外离线滤波、不降采样、不补零、不重写；控制位置本身包含在线低通、切换和恢复过渡，保留完整浮点精度。后续可以自行选择滤波器、展开yaw或重新解算姿态。图表只在数据无效或接收间隔超过0.5秒时断线，避免跨缺测区间连线。

## 文件结构

默认绝对路径是 `/home/enhe/uwb/log/experiments/`，每次实验创建 `<YYYYMMDD_HHMMSS>_<标签>_<唯一编号>` 子目录；默认标签为flight。`log/experiments_demo/`存放软件验证产生的合成数据，`log/ros/`存放ROS节点终端输出，二者都不是实际飞行的论文原始数据目录。

```text
log/experiments/<时间_标签_编号>/
├── uwb_filtered.csv            # 质量门限通过后的在线低通UWB XY
├── trajectory.csv              # 原始UWB位置
├── control_position.csv        # 控制器实际采用的位置和来源
├── position_sources.csv        # 里程计、UWB质量和切换过程
├── reference.csv               # 期望位置、map期望速度及最小导航状态
├── velocity_desired.csv        # 期望前/右速度
├── velocity_sent.csv           # 实际交给UDP的ID84值、有效性和输入年龄
├── velocity_actual.csv         # 实测前/右速度、原始高度与有效标志
├── attitude.csv                # 四元数、三轴角度、p/q/r
├── alignment.csv               # 每次对齐和参考失效事件的数值
├── events.csv                  # 航向、目标、控制状态变化，仅事件发生时写入
├── session.json                # 时间、开录触发方式、航向参考时间、文件行数及写盘状态
├── config/                     # 本次控制/UWB/场景/记录配置副本
├── parameters/                 # 可读到的控制、通信、UWB运行参数快照
├── summary.json                # 数据缺失及行数完整性提示
├── export_status.json          # 图表导出进度/错误
├── report.html                 # 小型预览页
├── report/                     # 总览、角度/速度等PNG及矢量PDF
│   ├── trajectory_uwb_raw.png / .pdf       # 单独原始UWB与期望参考
│   └── trajectory_uwb_filtered.png / .pdf  # 单独在线滤波UWB与同一期望参考
└── localization_comparison/    # 同次原始UWB/补偿定位两图、原始点与状态
```

各CSV的字段用途如下（所有文件都带共同时间列）：

| 文件 | 主要字段及含义 |
|---|---|
| uwb_filtered.csv | 在线低通XY，原始观测时间戳与Z；滤波参数见fusion_odometry.yaml |
| trajectory.csv | `x_m/y_m`为UWB驱动转换到map后的原始水平位置；`z_uwb_raw_m`仅保留UWB输出，不作为已验证高度；`frame_id`为坐标系 |
| reference.csv | `goal_x_m/goal_y_m`为控制器当时采用的期望位置；`vx_map_desired_mps/vy_map_desired_mps`为位置环输出的map速度；`trajectory_vx_mps/trajectory_vy_mps`为轨迹解析前馈速度；另有启用/输入有效/目标标志、控制状态、模式、阶段、任务运行时间与实际相位周期 |
| control_position.csv | `x_m/y_m`为控制器实际采用的位置；`valid`为有效性；`source`为来源；`assistance_enabled`表示是否启用光流辅助 |
| position_sources.csv | `selected_*`为定位选择器候选输出，`odom_*`为光流里程计，`raw_uwb_*`为原始UWB；还保留UWB/光流有效性、数据年龄、`innovation_m`创新量（UWB与里程计差异）和`uwb_syncs`同步次数。实际控制采用什么位置以control_position.csv为准 |
| velocity_desired.csv | `forward_mps/right_mps`为控制器计算的水平机体系期望速度，前正/右正；`valid`为有效性。它是桥接发送适配前的数据，不是飞控收到或执行的回执 |
| velocity_sent.csv | `forward_wire_mps/lateral_wire_mps/z_wire_mps`为打包后的ID84数值，包含输出Y符号适配；`valid`表示发送时输入通过检查，false时为零指令；另有frame_id、command_age_ms、attitude_age_ms和reason。存在记录表示成功交给本机UDP，不证明飞控已收到 |
| velocity_actual.csv | `forward_mps/right_mps`为ID250实测速度；`velocity_valid`为速度有效性；`height_up_m`为向上为正的高度（不是垂直速度）；`height_valid`为高度有效性；`device_time_us`为设备时间 |
| attitude.csv | 原始四元数`qw/qx/qy/qz`，`roll_deg/pitch_deg/yaw_fc_deg/yaw_map_deg`，机体角速度`p_rad_s/q_rad_s/r_rad_s`；另有姿态/角速度有效性、设备时间和重置计数。`yaw_fc_deg`为飞控自身参考航向，`yaw_map_deg`为对齐到map后的航向 |
| alignment.csv | 每次航向对齐或失效时的设备/电脑单调时间、`map_to_enu_yaw_rad`航向转换偏置、四元数约定`quaternion_body_to_ned`及有效标志 |
| events.csv | `event/detail`为航向对齐、目标改变、轨迹阶段、定位来源和控制状态变化等离散事件；不重复保存每个轨迹采样点 |

`source`可为`UWB`（正常UWB）、`FLOW_ODOMETRY`（光流接管）、`UWB_BLEND`（恢复过渡）、`INVALID`（无可用位置）。`parameters/`只包含当时能通过ROS参数服务读到的节点快照；缺少某节点文件表示没有成功获取其参数，不代表该节点参数为空。`config/`保留控制、UWB、基站、日志、光流定位和轨迹六份配置副本。

`session.json`记录实验起止时间、模式、辅助开关、文件行数及写入情况；`summary.json`记录导出时发现的缺失数据、行数不一致、丢弃行数和写盘错误；`export_status.json`记录图表导出阶段与错误。`report.html`可直接在浏览器打开；`report/trajectory.*`为期望/实际位置，`report/velocity.*`为前向/右向速度对比，`report/attitude.*`为三轴角度与p/q/r，均同时提供PNG和PDF。隐藏的`.export.lock`与`report/.matplotlib/`仅用于导出互斥及绘图缓存，不是实验测量数据。

`t_s`使用电脑接收单调时间，相对本次正式开录计时；轨迹模式t=0对应首个有效TRACKING状态，定点模式对应航向对齐。`receive_ros_ns`和`source_ros_ns`保留原始时间基准，飞控数据另有设备时间。不同CSV保持各自采样频率，适合后期离线时间配对。

CSV采用UTF-8 BOM，可以用Excel、LibreOffice、Python或MATLAB读取。状态与有效标志用0/1，缺失数值保持nan。配置和事件仅保留必要信息，原始CSV由独立写盘线程持续写入，退出时排空队列并关闭文件，再离线绘图；图表失败也保留CSV。

两次遮挡实验可用 `bash scripts/flight_log.sh compare-positions <无辅助目录> <有辅助目录>` 生成同坐标范围的两张位置图及相应CSV。方法与解释边界见 [光流接管实验](fusion_odometry.md)。

轨迹模式默认只从首次TRACKING开始记录，WAITING_START和首次APPROACH不会出现在位置/参考CSV中；已经开录后再次发起轨迹，则后续阶段仍按实际事件保留。轨迹中产生的连续参考只写入reference.csv，不把每个采样点重复写成航点事件。实际控制速度仍在velocity_desired.csv，解析轨迹速度另存reference.csv，二者不要混为同一个量。

## 查看、重画与异常恢复

日常不需要下面的命令来保存；它们只用于查看和后处理：

```bash
bash scripts/flight_log.sh list
bash scripts/flight_log.sh open
# 保留原始CSV，重新生成图表
bash scripts/flight_log.sh export log/experiments/<实验目录>
# 进程异常中断且已确认退出时，从已落盘CSV恢复图表
bash scripts/flight_log.sh export log/experiments/<实验目录> --recover
```

强制断电或SIGKILL不能保证最后缓冲区落盘，CSV行数检查会提示不完整；常规Ctrl+C会自动保存。不要反复按Ctrl+C打断绘图。导出支持同目录互斥锁和持久状态，正常导航关闭最多等待300秒用于收尾。

清理旧bag不会删除已有CSV和图表；旧格式从bag重新导出需要原bag仍然存在。新记录器完全不依赖rosbag写入，重画图表直接读取本次CSV。

配置位于 `src/flapping_uav_bringup/config/experiment_logging.yaml`，可设置轨迹开录时机、队列上限和自动绘图；启动参数 `log_root:=... experiment_label:=...`可改保存路径与标签。只改变日志，不改变位置环或飞控控制参数。

## 验证

```bash
bash scripts/build.sh
bash scripts/test.sh
bash scripts/ros2.sh python3 scripts/check_paper_logging.py
bash scripts/ros2.sh python3 scripts/check_localization_comparison.py
```

测试在独立DDS域中使用真实导航节点、回环MAVLink31/250和合成UWB，验证对齐前等待、拒绝过期对齐、自动开录、未滤波数值、角速度传递、NaN保留、目标记录、无bag、原有图、UWB滤波前后两图及同次定位对照两图，以及导航入口/导航子进程退出后的自动保存。离线测试还检查两图共用坐标范围、断流不连线、无效点不画成有效轨迹、原始CSV不改写、未接管/缺测时不伪称改善。没有访问实体飞控或遥控器。

在线滤波启用时，原始UWB与辅助定位两图包含滤波和可能的光流接管效果，不能将全部平滑改善归因于光流。纯滤波参考点另存uwb_filtered.csv，并复制到localization_comparison/uwb_filtered_points.csv；comparison.json记录online_uwb_filter，历史CSV保持不变。

自动轨迹开录验证：`bash scripts/ros2.sh python3 scripts/check_tracking_logging.py`。测试在私有DDS域使用合成导航状态，检查准备/接入不生成目录、有效TRACKING才开录、迟到的旧位置不混入、对齐元数据保留、定点/旧开关兼容。完整直线/八字合成飞行流程由scripts/check_trajectory_flight.py验证。

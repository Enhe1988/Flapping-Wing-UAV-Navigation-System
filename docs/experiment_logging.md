# 每次实验的数据记录与图表导出

记录节点为独立C++进程 `experiment_recorder_node`，只订阅实验数据，不发送飞控命令。磁盘写入使用有界队列和独立线程；图表在记录结束后由独立Python进程离线生成。原有导航/通信节点的控制逻辑和参数不变。

## 日常操作

仍按原流程启动通信及位置环。位置环入口现在默认自动启动记录，每次创建一个独立目录：

```bash
cd ~/uwb
# 终端1，保持运行
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py
# 终端2，保持运行；experiment_label可选，用于本次启动的文件夹标签
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py use_rviz:=true experiment_label:=flight01
```

每个实验目录位于 `log/experiments/日期_时间_标签_唯一编号/`，不会覆盖前一次实验。记录开始时会输出 `Recording experiment: ...`。

在另一个终端使用：

```bash
# 确认正在记录、当前目录、已接收/写入/丢弃消息数
bash scripts/flight_log.sh status
# 可选：为起飞、模式切换、动作、降落等阶段添加人工标记
bash scripts/flight_log.sh mark '起飞'
bash scripts/flight_log.sh mark '开始位置保持'
# 实验结束：停止本次记录，等待自动生成CSV、图表、PDF、HTML报告
bash scripts/flight_log.sh stop
# 用浏览器打开最新实验报告
bash scripts/flight_log.sh open
# 不重启控制节点，开始下一份独立实验记录
bash scripts/flight_log.sh start
```

`start/stop`只控制日志记录，不改变导航启用状态、目标点、遥控模式或飞机动力。正在记录时再次start不会重开或覆盖当前实验。日志停止后应先start再mark，避免将备注发到未记录的会话。

直接按Ctrl+C结束位置环launch也会自动收尾和生成报告。看到 `Experiment closed` 后再关闭终端；首次生成图表需要几秒。正常停止会先落盘原始bag，再生成图表，即使绘图失败原始数据也保留。

不要同时启动多个记录节点。若已单独启动记录器，位置环入口可加 `record_experiment:=false`。只启动独立只读记录器的命令是：

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup experiment_logging.launch.py experiment_label:=bench01
```

## 保存哪些数据

| 类别 | 话题/内容 |
|---|---|
| 定位 | `/uwb/position`、`/uwb/status`、原始解析后的Anchor_Frame0 |
| 独立融合观察 | `/fusion/odometry`、`/fusion/status`（节点运行时才有数据） |
| 控制 | `/navigation/status`、地图期望速度、两种目标请求话题、已接受目标及目标变化 |
| 机体速度/高度 | `/fc/body_motion`，保留valid与NaN |
| 速度指令 | 内部FRD `/fc/velocity_setpoint_body` 与实际发送 `/fc/velocity_setpoint_sent`，分开保存 |
| 姿态/对齐 | `/fc/attitude`、具有本次开机时间基准的 `/navigation/heading_alignment` |
| 遥控 | `/joy`、`/rc/override`的全部18通道 |
| 通信 | `/fc/mavlink_rx`、诊断计数、CRC/发送错误、消息速率、时钟/旧包过滤计数 |
| 事件 | `/rosout`、参数事件、`/experiment/mark`人工标记 |
| 参数 | 本次配置文件副本，以及从正在运行节点只读获取的参数快照 |

目前没有武断地根据高度或开关判断“已起飞”，请通过mark明确标记实验阶段。记录器没有收到的话题会在报告覆盖率表中显示0，不生成虚构测量值。

## 文件结构

```text
log/experiments/<本次实验>/
├── session.json                 # 起止时间、消息数、丢弃数、记录错误、话题类型
├── config/                      # 控制、UWB、基站、遥控、记录器配置副本
├── parameters/                  # 运行节点实际参数；读取时节点必须在线
├── bag/                         # 标准rosbag2 SQLite原始序列化数据与metadata.yaml
├── timeline.csv                 # 每条bag消息对应的电脑接收ROS时间及单调时间
├── csv/                         # 各话题原始字段、便于分析的派生表
├── events.csv / events.jsonl     # 状态切换、目标变化、异常、人工标记
├── summary.json                 # 消息覆盖、误差统计、异常次数与完整性提示
├── report.html                  # 自包含报告，可复制到别的电脑直接打开
└── report/
    ├── flight_report.pdf        # 多页图表PDF
    └── *.png                    # 9张独立图表
```

`dropped_messages`只统计记录器自身队列溢出的丢弃，不等同于网络或DDS丢包率；请结合源计数、接收间隔和通信诊断分析。

CSV使用UTF-8 BOM，适合Excel/LibreOffice读取。HTML内嵌PNG，不依赖外部网络、CDN或旁边的图片目录。保留整个实验目录便于在已构建的ROS2工作区重新绘图；只分享结果可复制HTML、PDF或PNG，查看它们不需要ROS。

9张图包括：场地/轨迹和目标、位置误差、前/右速度跟踪、内部与实际发送指令、姿态及map航向、高度与实际水平速度、输入年龄与控制状态、通信速率/错误、遥控CH1–8。CSV仍保存所有18通道。

## 时间、坐标与统计约定

- 横轴优先使用电脑接收单调时间，相对本次记录开始计时；保留原始header和设备时间，飞控时钟回退不会把整张图的横轴弄乱。
- 超过0.5秒的高频话题间隔以断线表示；1Hz通信诊断使用2秒断线阈值。不会把丢失区间插值成“平稳飞行”。无效速度、姿态、原始NaN不当作有效零测量。
- 速度跟踪图比较**内部前/右指令与前/右测量**。取反后的wire Y单独画在发送图里，避免将相反符号误判成速度环跟踪错误。
- ID250.z是向上高度；不把UWB Z当作已验证高度，也不将高度当垂向速度。
- map航向只在记录的对齐时间基准仍匹配该设备样本时显示；飞控复位后不会继续套用旧偏置。
- RMSE/最大误差只使用“已启用、输入有效、有活动目标且定位新鲜”的采样点，是采样统计，并非自动提取出的纯飞行阶段。完整实验含地面准备；分析特定阶段可按人工标记筛选CSV。
- 配置文件可能在节点启动后被修改，因此报告优先显示运行参数快照。离线/未启动的节点无法读取运行参数时，保留文件配置副本。标定偏置的运行值以heading_alignment记录为准。
- `velocity_setpoint_sent`仅证明电脑成功交给UDP的数值，不是飞控执行回执。

## 重新导出与异常中断

```bash
# 重新生成最新实验报告
bash scripts/flight_log.sh export
# 指定某次实验
bash scripts/flight_log.sh export log/experiments/<本次实验目录>
# 列出已有实验
bash scripts/flight_log.sh list
```

记录未结束时普通export会拒绝，避免把进行中的数据当作完整实验。若进程异常退出或电脑断电，确认记录器已经停止后可尝试只读恢复已提交的SQLite数据：

```bash
bash scripts/flight_log.sh export log/experiments/<异常实验目录> --recover
```

恢复报告明确标注异常中断与可能缺失的单调时间索引；不修改原始bag或伪装成完整实验。尚在内存队列、未提交的数据无法恢复。若后台记录队列满或磁盘写入失败，状态和报告会明确列出丢弃消息/错误。

自定义保存位置，在位置环或独立日志launch追加 `log_root:=/绝对路径`；离线工具使用 `--root /绝对路径`。记录配置位于 `src/flapping_uav_bringup/config/experiment_logging.yaml`，可设置自动开始、自动导出与队列上限。

原始bag可用于进一步分析。回放含目标、遥控、速度话题，请使用独立ROS_DOMAIN_ID，避免这些话题进入实机控制节点。

## 验证

```bash
bash scripts/build.sh
bash scripts/ros2.sh python3 scripts/check_experiment_logging.py
```

验证使用独立ROS域和合成数据，不访问真实串口、遥控器、飞控或UDP控制端点。验证报告、参数快照、原始NaN、航向会话、消息数一致性、多次实验及SIGINT自动导出。生成的示例在 `log/experiments_demo/`，与真实实验目录分开。

独立融合里程计运行时，报告另外生成 `fusion_comparison.png`，对比UWB与融合XY并显示推算/失效状态；它没有被选作位置环输入。详细说明见[融合观察器](fusion_odometry.md)。

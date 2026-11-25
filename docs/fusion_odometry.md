# UWB与光流里程计的位置接管

当前默认关闭光流接管（position_hold.yaml中的use_optical_flow_fallback: false），位置控制只使用通过质量检查、再经在线低通的UWB XY；UWB不可用时按原有恢复流程输出零。光流里程计继续观察和记录。以下接管与切回行为仅在显式设置use_optical_flow_fallback:=true时参与控制。

启用辅助时，位置控制优先使用滤波UWB XY。光流速度经本次航向对齐转到map，持续积分维护独立XY里程计，并定期用该滤波UWB同步。UWB跳变、失效、断开或时间戳冻结时，位置环自动使用光流里程计；UWB连续恢复并通过门限后，平滑切回UWB。

该功能已经接入位置环。飞控速度环、XY控制增益、遥控映射、消息84发送符号不因它改变。

## 日常启动

```bash
# 终端1：通信
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py
# 终端2：导航、UWB、光流里程计、位置选择、日志和RViz
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py use_rviz:=true
# 机头朝map +X，终端3执行
bash scripts/ros2.sh ros2 service call /navigation/start_here std_srvs/srv/Trigger '{}'
```

正常入口已经自动启动fusion_odometry_node，不要再启动第二个同名节点。start_here需要可靠UWB与有效姿态，成功后向里程计同步本次航向并自动开始论文CSV记录；贴地光流NaN不阻止UWB状态下开始位置保持。

## 状态与切换

`/localization/position`为带时效、会话和有效标志的位置选择输出，`/navigation/status.position_used`才是控制器实际使用的位置。

| position_source | 控制位置 |
|---|---|
| UWB | 原始质量检查通过后的低通UWB，不叠加光流积分 |
| FLOW_ODOMETRY | UWB不可用，使用有效光流连续推算的位置 |
| UWB_BLEND | UWB恢复后的过渡，逐步消除切换偏置，最后严格回到UWB |
| INVALID | 两路不足以提供有效位置，停止产生有效速度控制 |

状态通过可靠UWB年龄、光流年龄、创新、同步次数等辅助字段可查看：

```bash
bash scripts/ros2.sh ros2 topic echo /localization/position
bash scripts/ros2.sh ros2 topic echo /navigation/status
```

### UWB判断

每个新的UWB点检查数值、坐标确认、来源话题状态、时间、相邻可靠位置的位移/速度，以及相对光流预测位置的创新。拒绝的点不会改变最后可靠位置、延长推算时限或校正光流里程计。UWB状态按5Hz发布，位置约50Hz，两者使用不同的时效参数，避免误把状态发布间隔当作位置断流。

默认每0.2秒用可靠UWB同步一次里程计；每个点都检查，但不是每个点都重置积分。光流不可用而UWB正常时，位置仍由UWB维护，下一次有效光流需重新锚定，不能跨未观测的速度空档积分。

2026-11-25 05:18:35实验出现两次约240ms的原始UWB空档，原150ms超时会触发恢复确认；与光流不连续重叠时曾导致约0.62秒零输出。现将uwb_timeout_ms调为300，跳变/创新门限保持不变。超过300ms或质量被拒绝仍按原规则处理，不能据此保证所有链路条件下不中断。

### UWB在线滤波

默认启用一阶XY低通：`filtered += (1-exp(-dt/tau))*(raw-filtered)`，tau=0.10秒，dt为相邻接受观测的实际单调时间间隔。只在新点通过原始质量门限后更新一次；重复、拒绝、NaN、过期数据不更新，不用定时器伪造新的定位。首次初始化、超时后或完成异常恢复确认后，以当前可靠点重置滤波状态。时间常数越大越平滑，跟踪延迟也越大；先用0.10秒评估。

- `/uwb/position`：保留驱动坐标转换后的原始UWB，用于质量判断、日志和原始轨迹。
- `/uwb/position_filtered`：仅接受的低通XY，保留该次原始观测时间戳，Z与协方差原样保留，不声称噪声方差已重新标定。
- `/localization/position`：正常使用滤波UWB，异常时按原有规则切换光流里程计。

正常position_hold入口在辅助开/关两种模式都运行定位节点。辅助关闭时，控制器仍使用`/uwb/position_filtered`，但不会采用FLOW_ODOMETRY；UWB质量拒绝或断流时按原有输入恢复流程输出零。两组都有相同滤波与质量门限，避免把滤波收益误算为光流收益。单独ros2 run控制器的历史默认输入仍为`/uwb/position`，可用参数uwb_position_topic选择新话题；日常launch已自动设置。

### 恢复

恢复点必须连续满足检查，默认至少10个点、持续0.5秒；任一坏点重新开始确认。不能只因连续收到某个大偏移位置就判定恢复正常。确认期间仍用光流，之后0.5秒平滑切回，避免直接把两路差异送入位置环。

切入光流时也保持位置连续。切换不改目标；只有原有故障处理确实清除了目标，才需要重新start_here。

### 有效范围

- 默认最多从最后一个可靠UWB起推算5秒，包含断流确认时间；即使一直收到坏点也不续期。
- 光流NaN、冻结、超时、时间空档、姿态失效或航向会话不匹配时，不能用假零速度继续推算。
- 到达推算上限或两路失效时，位置环输出零并进入原有2秒恢复等待；持续失效则清除目标。飞控参考重置仍立即清除目标，重新对齐。
- 恢复位置超过创新门限时继续拒绝，不能承诺在任意漂移后自动重建绝对坐标；停机检查后可用 `/fusion/reset` 重建初值。
- 这是松耦合的速度积分、位置校正和来源切换，不是测距级融合、光流偏置估计或已标定的精度保证。缓慢UWB偏置可能通过门限，光流也会漂移，需用真实实验检验。

## 论文遮挡对照实验

同一次实验的定位对照已自动接入日志：正常启动并对齐，在实验中短暂遮挡UWB，Ctrl+C结束导航后，localization_comparison/生成原始UWB与在线补偿估计两张图，各有PNG/PDF和原始点CSV；两图共用坐标与时间范围。补偿数据是同次/localization/position位置选择器实际输出，并非离线虚构另一架次轨迹。有效接管样本、UWB未接受时段和实际控制辅助开关记在comparison.json，详细字段见 [日志说明](experiment_logging.md)。未产生坏UWB或有效光流接管时，两图可能接近相同，不能预设一定出现改善。

启用在线滤波后，原始/补偿图同时体现滤波与可能的光流接管。uwb_filtered.csv额外保留纯UWB滤波参考点，comparison.json记录online_uwb_filter。若要单独证明光流接管效果，应让两组使用相同滤波配置，或结合该参考数据分析，不能把所有变平滑都归因于光流。

以下保留两次独立飞行的控制表现对照方法，适用于进一步验证开关辅助对闭环飞行的影响。

分两次真实实验，保持场地、基站标定、目标、增益与遮挡操作尽量一致。当前默认关闭辅助；两组通过启动参数显式选择开关，不改其他控制参数。

```bash
# 第一组：UWB-only，仍有原来的数值/超时与故障保护
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py \
  use_rviz:=true use_optical_flow_fallback:=false experiment_label:=uwb_only
# 对准+X调用start_here，完成实验后Ctrl+C自动保存

# 第二组：开启光流接管
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py \
  use_rviz:=true use_optical_flow_fallback:=true experiment_label:=flow_assisted
# 同样对齐、实验、Ctrl+C自动保存
```

两组不能同时运行。通信入口可保持运行。也可叠加flight_mode:=line或figure8；两组必须采用同一轨迹配置，分别在对齐后手动start_trajectory。先核对遮挡时实际产生的是跳变、缺失还是仍正常；不能假设遮挡必定产生大跳变。缺失数据画断线，算法没有产生有效输出时不伪装成平稳位置。

每次自动生成的trajectory.png/pdf都包含期望位置、原始UWB和实际控制位置。原始UWB、积分里程计和控制使用的位置分别保存，不相互覆盖：

- trajectory.csv：收到的UWB map位置，不经本模块事后修改。
- uwb_filtered.csv：原始门限通过后在线滤波的UWB XY，保留观测时间戳和原始Z。
- position_sources.csv：原始UWB、光流里程计、质量检查、候选来源和时效。
- control_position.csv：该组控制器实际采用的位置、有效标志、来源及辅助开关。
- events.csv：位置来源变化、导航状态和目标变化。

两次实验结束后可一键生成统一坐标范围的两张论文位置对照图，并复制相应原始位置CSV：

```bash
bash scripts/flight_log.sh compare-positions \
  log/experiments/<uwb_only实验目录> \
  log/experiments/<flow_assisted实验目录>
```

结果在第二组目录的position_comparison/，包含uwb_only_position.png/pdf/csv与flow_assisted_position.png/pdf/csv。它只使用两次真实记录，不生成“假想的无辅助飞行”，不离线滤波、改点或插值。文件中的两组模式必须与实测控制开关匹配。

位置曲线展示定位估计的稳定性与连续性；没有独立真值时，不把估计平滑直接当作真实飞行位置精度。工程自带故障注入验证的图来自合成数据，只用于检查软件，不用于当作实飞论文结果。

## 参数

文件为 [fusion_odometry.yaml](../src/flapping_uav_bringup/config/fusion_odometry.yaml)，以下是可调整的初始值，尚需实测整定。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| uwb_timeout_ms | 300 | 新位置最长等待，覆盖实测240ms短空档 |
| uwb_filter_enabled | true | 对通过原始质量门限的XY启用低通 |
| uwb_filter_tau_s | 0.10 | 时间常数，单位秒，增大后更平滑但延迟增加 |
| uwb_status_timeout_ms | 500 | 5Hz UWB状态时效 |
| flow_timeout_ms | 400 | 速度保持有效窗口 |
| attitude_timeout_ms | 500 | 姿态时效 |
| uwb_sync_period_s | 0.2 | 可靠UWB同步里程计周期 |
| max_dead_reckoning_s | 5.0 | 最后可靠UWB之后的推算上限 |
| uwb_jump_slack_m | 0.3 | 位移门限固定项 |
| max_uwb_speed_mps | 3.0 | 位移门限的速度项 |
| uwb_innovation_gate_m | 0.75 | UWB与预测位置允许偏差 |
| uwb_recovery_confirmation_samples | 10 | 连续恢复点数 |
| uwb_recovery_confirmation_s | 0.5 | 连续恢复时长 |
| uwb_recovery_blend_s | 0.5 | 切回过渡时长 |

正常控制的默认辅助开关在position_hold.yaml的use_optical_flow_fallback；launch的true/false可覆盖它。参数修改后重启导航并重新对齐。

## 独立观察与坐标

仍可在没有正常导航入口时单独启动fusion_odometry.launch.py，使用 `/fusion/align_positive_x` 独立对齐并在RViz看积分轨迹。正常飞行请用 `/navigation/start_here`；导航航向优先。

ID250使用前正、右正的水平速度，经姿态与本次航向旋转到map。输出消息84的发送符号参数不用于积分（当前velocity_y_sign=1）；ID250.z是高度，不当作vz。仅估计XY。`/fusion/odometry`表示积分里程计，child_frame_id=fusion_map_axes，其轴平行map、单位姿态不表示飞机机头；不发布额外TF。协方差是模型值，不是已标定精度。

## 软件验证

```bash
bash scripts/build.sh
bash scripts/test.sh
bash scripts/ros2.sh python3 scripts/check_position_fallback.py
```

故障注入验证使用独立DDS与合成UWB/光流，运行真实位置控制器，检查两组模式、定时同步、跳变隔离、超过2秒的短时接管、恢复确认/过渡、光流失效、5秒上限、持续故障锁存、飞控参考重置、自动CSV与两张位置图。不会访问硬件或将合成结果冒充实飞验证。

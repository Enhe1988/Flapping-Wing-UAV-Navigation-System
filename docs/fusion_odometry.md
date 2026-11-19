# 独立UWB＋光流XY融合里程计（观察版）

此功能用于先看效果：独立C++节点 `fusion_odometry_node` 读取已有UWB、光流速度和姿态，输出单独的XY里程计。**没有替换 `/uwb/position`，没有修改位置环输入，没有发布飞控速度/遥控/目标指令，也没有加入正常启动入口自动运行。**

## 运行

先按现有方式让 `/uwb/position`、`/uwb/status`、`/fc/attitude`、`/fc/body_motion` 有数据。随后另开终端：

```bash
cd ~/uwb
bash scripts/ros2.sh ros2 launch flapping_uav_bringup fusion_odometry.launch.py use_rviz:=true
```

如果现有位置环已经发布了本次有效 `/navigation/heading_alignment`，观察节点可以只读借用；也可以完全独立地对齐：将机头朝map +X（当前A0→A3），执行：

```bash
bash scripts/ros2.sh ros2 service call /fusion/align_positive_x std_srvs/srv/Trigger '{}'
```

**这个服务只记录本节点的航向，不启用位置环，不发送飞控指令，不改变导航目标。** 它不依赖光流在地面是否有效，也不依赖UWB当时是否可用。自己记录过航向后优先保留独立参考，不再被导航对齐消息覆盖；飞控重启后重新调用该服务。

查看：

```bash
bash scripts/ros2.sh ros2 topic echo /fusion/status
bash scripts/ros2.sh ros2 topic echo /fusion/odometry
```

不想启用位置控制时，可以使用原有UWB独立驱动及只接收的MAVLink桥接，观察节点不要求位置控制器运行。只接收桥接示例（不要与已绑定9000端口的通信节点重复启动）：

```bash
bash scripts/ros2.sh ros2 run flapping_uav_gcs mavlink_bridge_node --ros-args -p tx_enabled:=false -p send_velocity:=false
```

## 算法

1. 将ID250的水平前向/右向速度，使用回传姿态与本次航向偏置旋转到右手map平面。
2. 按电脑单调接收时间，在有效速度的短期保持窗口内做 `p += v_map * dt` 预测。重复设备时间不刷新速度；大设备时间间隔、失效姿态和无效速度中断预测，不跨未知空档补积分。
3. UWB通过坐标确认、时效、步长/速度门限、相对预测位置的创新门限后，执行位置校正。
4. 默认校正增益1.0：每个接受的UWB点直接校正到其XY；两次UWB之间由光流预测。增益减小才是平滑校正，不能再声称每次都与UWB完全重合。
5. UWB跳变/失效/断开时，不刷新最后可靠UWB时间，允许从该时刻起最多2秒的连续光流推算；超时或光流也失效，停止发布新的有效里程计。
6. 首次初始化、异常后的UWB恢复需连续3个通过门限的新位置。大偏差不会因“连续收到”就自动放行；需要核查数据、调整观察门限或人工重置。

这是位置校正＋速度预测的松耦合互补观察器，**不是紧耦合EKF**。不估计光流比例/偏置，不做UWB测距级融合，没有硬件时钟同步或历史延迟回溯。世界系速度按电脑接收/处理时间进行有界零阶保持积分，设备时间用于检查新鲜度、先后顺序和姿态配对；暂不把它当作可直接用于自主飞行的定位方案。

## 状态与失效边界

| state | 含义 |
|---|---|
| WAITING_UWB | 等待连续通过检查的UWB建立绝对XY初值 |
| UWB_ONLY | UWB仍可用，光流/姿态/航向条件不足，位置由UWB维护 |
| FUSED | 正常进行光流预测和UWB校正 |
| DEAD_RECKONING | UWB不可用或被拒绝，正在进行有时限的光流推算 |
| INVALID | 推算超时、光流不可用或存在未观测区间，位置不再作为有效输出 |

`/fusion/status.valid`是当前估计有效标志，含UWB/光流/姿态年龄、配对时间差、UWB创新、接受/拒绝/重复计数与光流间断计数。`propagation_complete=false`说明存在不能补回的速度空档，不能仅凭后来速度变好就把旧位置重新当作有效；需新UWB重新锚定。

- UWB可靠时，光流NaN不会使位置失效，进入UWB_ONLY即可。
- UWB也失效时，无效光流**不会假装成真实零速度**；不发布假新鲜的位置。
- 2秒从最后一个接受的UWB开始计时，包含等待判定UWB过期的时间；坏点流不会无限续期。
- 无效期间 `/fusion/odometry` 与 `/fusion/position` 不再刷新；`/fusion/status`继续发布，RViz灰点为最后估计且标明invalid。外部程序不能只看历史最后一条Odometry判断当前有效。
- UWB恢复后相对估计偏差超过门限会继续拒绝。检查确定需要重建初值时：

```bash
bash scripts/ros2.sh ros2 service call /fusion/reset std_srvs/srv/Trigger '{}'
```

reset只重建观察器XY初值和轨迹，等待下一组确认UWB，不清除/改变实际导航目标。移动基站重标定后还要同步更新基站显示与坐标约定，并重新朝新+X记录航向。

## 坐标、Z和协方差

- 输入位置使用 `/uwb/position` 的map坐标，已含原始UWB Y反向，不会再次反向。
- ID250仍按前正、右正处理；之前发送ID84的Y取反只属于输出适配，不用于此处的测量积分。
- 仅估计XY；ID250.z是高度，不当作vz积分。Odometry的Z=0只是未估计占位，Z及姿态/角速度协方差设大值。
- Odometry的 `child_frame_id=fusion_map_axes` 是原点位于估计点、坐标轴平行map的虚拟帧；姿态单位四元数不代表飞机朝向，twist XY也表达在这个与map平行的帧中。没有发布TF，避免影响已有姿态/定位关系。
- XY协方差可由有效UWB协方差重置，否则用配置初值；预测期间按模型增长。`covariance_is_model=true`明确它不是已标定的精度保证。

RViz配置 `fusion_comparison.rviz` 同时展示原始UWB轨迹和融合轨迹。融合点正常为青色、短时推算为黄色、无效为灰色；没有发布目标点的工具。轨迹最多3000点，5Hz刷新，位置与状态默认50Hz。

## 参数

文件：`src/flapping_uav_bringup/config/fusion_odometry.yaml`，以下为观察初值，并非飞行整定结果。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| uwb_timeout_ms | 500 | UWB位置/状态新鲜度 |
| flow_timeout_ms | 300 | 有效光流速度最长保持时间 |
| attitude_timeout_ms | 300 | 姿态新鲜度 |
| max_attitude_pair_skew_ms | 250 | 速度与姿态设备时间最大差 |
| max_flow_device_gap_ms | 300 | 更大的速度设备时间空档需重新锚定 |
| max_dead_reckoning_s | 2.0 | 从最后可靠UWB起的最大推算时长 |
| uwb_jump_slack_m | 0.3 | 步长门限的固定容差 |
| max_uwb_speed_mps | 3.0 | 步长门限的速度项，允许步长=容差+速度×间隔 |
| uwb_innovation_gate_m | 0.75 | UWB与预测位置的最大允许差 |
| uwb_confirmation_samples | 3 | 初始化/恢复的连续确认样本数 |
| uwb_correction_gain | 1.0 | 接受UWB时的校正比例 |
| max_flow_speed_mps | 5.0 | 光流速度数值门限 |

修改配置后重启该观察节点即可，不需要改位置环。超过门限只能说明“未通过当前检查”，不能证明数据一定错误；同样，通过检查也不代表UWB没有缓慢偏置。

## 实验记录

已有实验记录器会自动订阅 `/fusion/odometry`、`/fusion/status`，并保存运行参数。导出时增加 `fusion_comparison.png`，对比UWB与融合XY、标注短时推算区间和模式变化；普通无融合实验仍为原来的9张图。独立观察时可另开记录器：

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup experiment_logging.launch.py experiment_label:=fusion_check
# 实验后
bash scripts/flight_log.sh stop
bash scripts/flight_log.sh open
```

不要重复启动已有记录器。此处只是记录和展示，不会让位置环改用融合结果。

## 验证

已完成编译，回归检查共70项、零失败。独立DDS域内的模拟实验验证了UWB跳变拒绝、2秒推算上限、无效数据停止更新、连续确认恢复、光流NaN与时间空档、飞控参考重置，以及10张图的日志导出；没有访问硬件或发布控制指令。这些结果不代表已经验证实飞定位精度。

复现模拟实验：

```bash
bash scripts/ros2.sh python3 scripts/check_fusion_observer.py
```

每次运行会把实际报告路径和验证结果写入 `log/fusion-observer-validation.json`；生成数据属于本地测试产物，不提交到Git。

# 当前精简飞控接口与启动流程

本文为0x8001速度模式参考。当前默认已经改成[直接姿态控制](direct_attitude_control.md)：0x8002角度指令、每轴±30°，与用户维护的飞控dfe1d83接口一致，注意关闭互斥的QGC调试回传。仅回归速度模式时，通信和导航同时传horizontal_control_mode:=velocity。

已按飞控仓库[74d6a5c](https://github.com/HuangGL03/XUSTBfly_control/commit/74d6a5cb6cd8beaa4970d135d74738a4cea5a176)适配电脑端。以实际代码为准：yaw为5Hz，扩展类型0x8001，payload首字节是版本1。此前方案中的20Hz、0x8000和长度字节不适用于该飞控版本。

## 网络与报文

备用入口仅桥接，不转发遥控：电脑监听0.0.0.0:14550，向10.0.0.1:14555发送。MAVLink系统/组件为电脑255/190、飞控1/1。

| 方向 | 消息 | 实际内容与频率 |
|---|---|---|
| 飞控→电脑 | ID254 DEBUG | ind=1；value为yaw(rad)，time_boot_ms为实际姿态采样时刻；5Hz、21字节 |
| 电脑→飞控 | ID248 V2_EXTENSION | message_type=0x8001；target_network=0、目标1/1；10Hz、最多27字节 |
| 双向 | ID0 HEARTBEAT | 各1Hz |

ID248的249字节payload先清零，再将前10字节写成小端`<BBhhI`：版本1、enabled(0/1)、vx(mm/s，前正)、vy(mm/s，右正)、最近有效yaw的time_boot_ms原值。两个速度各int16；0.5m/s编码为500。MAVLink 2自动截断尾零，实际帧长变化是正常现象。停机/失效时enabled=0且速度为0，不自动退回发送ID84。

桥接同时检查ROS指令是否新鲜，以及最近收到的有效yaw是否新鲜；yaw缺失、NaN、错误来源、错误ind、重复/孤立乱序、明显不合理的前跳不能续期。未收到yaw时仍发送禁用零帧，echo=0。DEBUG的采样时钟确认重启时，导航清除航向与目标，需重新start_here。

当前飞控将echo存入`pc_data.velocity_echo_ms`，尚未按echo拒绝过期命令。飞控当前指令接收超时为1500ms。电脑仍保留200ms发布看门狗与既有yaw时效检查，但它们不能证明无线链路已及时送达；减小报文不能保证消除此前4秒延迟。

## 常用启动

先停止之前运行的通信、导航和速度测试程序，再使用重新构建的环境。消息接口增加了yaw_only/yaw_rad字段，旧进程不能混用。

```bash
cd ~/uwb
# 终端1：仅上下位机通信
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication_backup.launch.py horizontal_control_mode:=velocity
# 终端2：直接速度阶跃测试，不运行导航、不需要UWB
bash scripts/ros2.sh ros2 run flapping_uav_gcs velocity_test.py
# 测试程序中输入：0.3 0 3；stop立即回零，q/Ctrl+C退出回零
```

直接速度测试不进行姿态转换，但当前精简桥接仍需接收到yaw，才能校验其时效并生成真实echo；不能通过伪造echo绕过。测试程序和导航同时存在时会拒绝继续发布。

完成链路检查后退出速度测试，再启动正常导航：

```bash
# 定点/手动航点；其他模式替换为flight_mode:=line或figure8
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py horizontal_control_mode:=velocity use_rviz:=true
# 机头朝map +X后，在另一个终端对齐
bash scripts/ros2.sh ros2 service call /navigation/start_here std_srvs/srv/Trigger '{}'
# 选了line/figure8时，对齐后先保持当前位置，手动开始轨迹
bash scripts/ros2.sh ros2 service call /navigation/start_trajectory std_srvs/srv/Trigger '{}'
```

对齐和目标发布接口不变。位置控制默认仅使用滤波UWB，保留P增益、限速、限加速度及2秒短时恢复；不再等待ID250。缺少实测速度时仅支持纯P，kd_x/kd_y必须为0，到达改用位置误差和驻留时间，actual_speed_mps为NaN而不是假设实测速度为0。光流速度环和定高仍在飞控本地运行。

```bash
# 查看yaw；yaw_only=true，yaw_rad有值，四元数/角速度为NaN
bash scripts/ros2.sh ros2 topic echo /fc/attitude
# 查看实际打包的速度；reason包含ID248、enabled与yaw_echo_ms
bash scripts/ros2.sh ros2 topic echo /fc/velocity_setpoint_sent
# 查看导航有效性、目标与当前控制位置
bash scripts/ros2.sh ros2 topic echo /navigation/status
```

飞控debug观察`pc_data.vx_set_mps/vy_set_mps`、`velocity_rx_count`、`velocity_update_ms`和新增`velocity_echo_ms`。最新飞控的光流模式以B中档/C初始档进入；进入后C中档允许右杆接管，C初始/第三档采用电脑水平速度。B第三档/C初始档可切回手动RUN，模式细节见上述飞控提交。

## 日志和RViz

航向对齐后自动记录，Ctrl+C结束导航后自动导出位置、期望速度和yaw。attitude.csv新增yaw_only/message_id；未回传的四元数、roll/pitch及角速度记NaN，velocity_actual.csv保留表头且无假造样本。图中明确标注未回传的观测。RViz只显示航向的水平模型并标注Heading only，不能用于核对真实倾斜。

光流里程计接管继续关闭；纯UWB过滤保留。精简模式不支持开启光流接管，否则启动报错，避免不完整的遥测进入控制。完整遥测下的论文速度跟踪和光流补偿实验需切回与旧接口相符的飞控固件及配置。

## 参数和回归验证

当前配置在position_hold.yaml：控制器`compact_yaw_only: true`、`use_optical_flow_fallback: false`，桥接`velocity_protocol: compact`、`attitude_message_id: 254`。正常入口无需新增参数。

保留旧固件回归入口：通信追加`velocity_protocol:=legacy`，导航追加`compact_yaw_only:=false`，仅在实际飞控仍发送ID31/250并接收ID84时使用；不进行自动协议猜测。

```bash
# 私有ROS域、本机UDP、合成UWB，无硬件访问
bash scripts/ros2.sh python3 scripts/check_compact_communication.py
```

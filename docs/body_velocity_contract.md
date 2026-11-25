# 飞控数据与速度接口

当前默认使用[直接姿态控制](direct_attitude_control.md)，角度与速度采用独立消息/话题，不能互相解释。本文仅描述显式选择velocity模式时的速度接口。

此前的速度协议对应飞控74d6a5c：ID254回传yaw，ID248发送速度，详见[精简速度接口](compact_mavlink.md)。本文其余ID31/250→84内容保留为legacy完整遥测接口参考。

日常链路由 `communication_node` 接收遥控和飞控数据，`position_controller_node` 在电脑上完成位置环及速度坐标转换，通信桥接打包发送。`velocity_transform_node`保留为可独立测试的通用转换组件，不参与日常位置控制链路。

## 回传

目标来源为system/component 1/1，UDP接收绑定0.0.0.0:9000。

| MAVLink | ROS话题 | 含义 |
|---|---|---|
| 0 HEARTBEAT | `/fc/mavlink_rx` | 原始心跳；当前控制配置仅作诊断 |
| 31 ATTITUDE_QUATERNION | `/fc/attitude` | WXYZ映射为ROS四元数字段，默认解释为body FRD→local NED |
| 250 DEBUG_VECT | `/fc/body_motion` | x前向m/s、y水平右向m/s、z向上高度m |

ID250的XY已由飞控补偿倾斜，所以电脑只按航向旋转水平速度，不再次做roll/pitch补偿。Z是高度，绝不当作垂向速度。原始NaN及有效标志保留在ROS消息和日志中。

设备时间必须持续前进。桥接过滤重复和孤立迟到数据；确认时间参考重置后，位置环清除目标与旧航向。数值有效不代表航向已经与map对齐。

本工程ID250时间字段使用与ID31相同的启动时钟（uint32毫秒乘1000），不按Unix时间解释。超出启动时钟范围或相对主机时间不合理前跳的样本会在更新基准前丢弃，防止单帧坏时间戳导致后续正常包被误判成飞控重启。

## 位置环与坐标转换

```text
/uwb/position + /navigation/goal + /fc/body_motion
  → map系 P/PD + 速度/加速度限制
  → 根据 /fc/attitude 和本次航向偏置旋转
  → /fc/velocity_setpoint_body
  → MAVLink桥接打包消息84
```

map +X/+Y来自UWB转换后的右手地图。机头朝map +X时调用 `/navigation/start_here`，记录本次航向并启用当前位置保持。

若机头在map平面的航向为ψ，世界期望速度为(vx,vy)，内部水平机体速度为：

```text
v_forward =  cos(ψ) * vx + sin(ψ) * vy
v_right   =  sin(ψ) * vx - cos(ψ) * vy
```

内部frame为 `base_link_heading_frd`，X前正、Y右正，单位m/s。只使用水平航向，不是倾斜机体完整三维FRD转换。

## 发送

发送目标为192.168.1.151:9111。日常入口开启心跳0、遥控70和水平速度84，飞控负责模式仲裁、速度/姿态内环及自身输入超时。

- ID84 `coordinate_frame=8`，`type_mask=3559`；仅启用vx/vy，忽略位置、垂向速度、加速度和yaw。
- 依据2026-11-25通信恢复后的样机方向核对，当前打包为 `vx_wire=v_forward`、`vy_wire=v_right`，`velocity_y_sign=1`。移除旧固件调试时的Y取反，不修改回传ID250的方向。
- `/fc/velocity_setpoint_body` 显示内部前/右指令；`/fc/velocity_setpoint_sent` 显示成功交给UDP的实际打包数值，frame为 `fc_velocity_command`。
- 该样机应用约定不能直接当作其它飞控的通用接口。反馈ID250的右正含义不跟随输出Y符号一起改变。
- 无效或过期指令不无限保持非零值；桥接本机指令超时为200ms，输出零XY。遥控输入失效时停止70，使用既有飞控超时行为。

UDP发送成功不是飞控执行确认。具体增益、超时和符号位于 [position_hold.yaml](../src/flapping_uav_bringup/config/position_hold.yaml)。正常操作见 [位置控制](position_hold.md)。

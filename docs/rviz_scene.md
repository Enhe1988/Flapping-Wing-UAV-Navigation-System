# RViz：基站、定位与飞机姿态

正常飞行入口可同时开启显示：

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup position_hold.launch.py use_rviz:=true
```

它已经启动UWB驱动、定位处理和场景节点。若只关闭了RViz，重新打开窗口即可，不要重复启动这些节点：

```bash
bash scripts/ros2.sh ros2 run rviz2 rviz2 -d "$(pwd)/src/flapping_uav_bringup/rviz/uwb.rviz"
```

## 不接飞控，只看UWB

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup uwb.launch.py   port_name:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0   baud_rate:=921600 coordinates_confirmed:=true use_rviz:=true
```

多行命令的反斜杠必须在行末。串口不能同时被NAssistant或另一个驱动占用。串口权限按本机设备组配置；加入dialout后需重新登录，避免每次开放所有用户写权限。

若定位驱动已运行但没有场景节点，可单独启动：

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup scene.launch.py
```

已有场景时只重新打开RViz窗口。若没有通信节点而需要查看姿态，可用 `scene.launch.py start_bridge:=true` 启动只接收桥接；已有9000接收器时不要重复开启。

## 显示含义

| 内容 | 含义 |
|---|---|
| 四个橙色方块 | scene.yaml配置的基站坐标，当前只确认XY |
| 红/绿/蓝坐标轴 | 右手派生系X/Y/Z，另外标识原始UWB的+Y |
| 绿色点与蓝色轨迹 | 当前UWB定位与历史位置 |
| 飞机模型和机体坐标轴 | 飞控完整四元数姿态，保留横滚和俯仰 |
| 橙色目标标记 | 位置环已接受的目标点 |

Fixed Frame使用 `map`。基站原始UWB坐标见 [scene.yaml](../src/flapping_uav_bringup/config/scene.yaml)，变换见 [uwb.yaml](../src/flapping_uav_bringup/config/uwb.yaml)。方向约定见 [UWB坐标说明](uwb_map_alignment.md)。

未记录本次导航航向时，飞机姿态显示在黄色独立预览区，不代表飞机的地图位置。调用 `/navigation/start_here` 成功后，场景读取 `/navigation/heading_alignment`，将完整姿态叠加到Tag位置；不需要把偏置写入文件。当前飞机模型Z=0.25m仅用于XY显示，不是实际高度，尚未补偿Tag到重心的偏移。

无效、过期或冻结的姿态会删除飞机模型。若没有点或飞机，先检查输入，不要只凭TF缓存判断数据有效：

```bash
bash scripts/ros2.sh ros2 topic echo /uwb/status
bash scripts/ros2.sh ros2 topic echo /fc/attitude
bash scripts/ros2.sh ros2 topic echo /diagnostics
```

## 融合观察窗口

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup fusion_odometry.launch.py use_rviz:=true
```

该窗口对比UWB与独立融合轨迹。融合点正常青色、短时推算黄色、无效灰色。这个配置没有发送目标的工具；正常目标操作使用uwb.rviz的2D Goal Pose或 `/navigation/goal`。详见 [融合观察器](fusion_odometry.md)。

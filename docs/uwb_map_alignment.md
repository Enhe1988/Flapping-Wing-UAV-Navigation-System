# UWB坐标与ROS map

当前场地中，站在A0朝A3看，A1在右侧。原始UWB +X沿A0→A3，+Y向右；ROS map使用右手坐标，+Y向左。转换由 [uwb.yaml](../src/flapping_uav_bringup/config/uwb.yaml) 配置：

```text
x_rh = x_uwb
y_rh = uwb_y_sign * y_uwb       # 当前-1
p_map = Rz(uwb_to_map_yaw_rad) * p_rh + uwb_to_map_translation_m
```

当前yaw和平移为0，因此 `map_x=uwb_x`、`map_y=-uwb_y`。目标点、位置环、融合观察器和RViz使用转换后的map，原始协议与 `/uwb/status.raw_position` 保留原值。

反射不能用四元数旋转表示。场景使用右手派生frame `uwb_rh`；先反向原始基站Y，再由静态TF描述正常旋转和平移。额外箭头标明原始UWB正Y方向，标签保留原始UWB数值。

## 基站移动后

1. 在UWB上位机或设备侧重新标定并保存实际基站坐标。
2. 将同一组原始UWB坐标填入 [scene.yaml](../src/flapping_uav_bringup/config/scene.yaml) 的 `anchor_positions_m`，顺序由 `anchor_ids` 决定，每站填写XYZ三项，单位米。
3. 若坐标轴方向或地图原点改变，同步核对uwb.yaml中的Y符号、旋转和平移，并重新启动相关节点。
4. 飞机朝新的map +X，调用 `/navigation/start_here`；独立观察则调用 `/fusion/align_positive_x`。

scene.yaml只控制显示，不能代替硬件标定。Z未测量时保持 `anchor_z_configured=false`，其中0只是XY投影占位；当前位置环不使用UWB Z。

## 航向

飞控每次上电会重建yaw参考。导航航向通过服务在当前会话内记录，不保存过期偏置供下次上电复用。当前场地map +X沿A0→A3；若更改UWB→map旋转，就应朝旋转后的map +X对齐。

位移转换与航向对齐是两件事：修改yaw偏置不能修正XY镜像，修改发送ID84的Y符号也不能改变UWB坐标。

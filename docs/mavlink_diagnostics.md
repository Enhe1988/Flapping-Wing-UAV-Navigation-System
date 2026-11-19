# 通信与遥控诊断

当前链路为图像板192.168.1.151:9111 → 电脑192.168.1.100:9000，电脑控制消息发回192.168.1.151:9111。端口以通信launch参数和设备实际配置为准。

## ROS内查看

已有通信节点时，直接订阅，不再开第二个UDP接收器：

```bash
bash scripts/ros2.sh ros2 topic echo /fc/attitude
bash scripts/ros2.sh ros2 topic echo /fc/body_motion
bash scripts/ros2.sh ros2 topic echo /fc/mavlink_rx
bash scripts/ros2.sh ros2 topic echo /rc/override
bash scripts/ros2.sh ros2 topic echo /diagnostics
```

只接收飞控、不发送任何UDP消息且不启动遥控器：

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py   tx_enabled:=false use_joy:=false verify_device:=false
```

诊断包含消息速率、发送错误、旧包/重复包丢弃及设备时钟重置计数。单条 `old/reordered sample dropped` 表示迟到包已被过滤，不等同于链路完全断开。

查看位置环真正发送的速度用 `/fc/velocity_setpoint_sent`，取反前内部指令用 `/fc/velocity_setpoint_body`。输入消息或时间戳停止更新，不能仅凭终端中最后一个非零数值判断仍在接收。

## 独立UDP监视

`scripts/inspect_mavlink.py`不需要ROS，不发送请求或控制消息。只在正式通信桥接停止时监听同一端口。可选诊断依赖pymavlink不属于C++运行链路：

```bash
sudo apt install python3-venv
python3 -m venv /tmp/uwb-mavlink-debug
/tmp/uwb-mavlink-debug/bin/pip install pymavlink
/tmp/uwb-mavlink-debug/bin/python scripts/inspect_mavlink.py --ports 9000 --duration 60
```

默认端口为9000。其他端口可通过 `--ports` 指定。输出 `log/mavlink-inspection-summary.json` 与逐帧JSONL，包含本地端口、远端来源、消息ID、数量和最后字段；非有限数保留为字符串。它只看到列出的绑定端口，未解码不代表没有网络数据。

## 抓包与USB遥控检查

```bash
# 本机终端输入sudo凭据；默认抓所有网卡20秒，不主动发送数据
bash scripts/capture_ground_link.sh
# 或指定实际网卡
bash scripts/capture_ground_link.sh 网卡名称
# 独立DDS域内只读检查Pocket的原始/joy输入
bash scripts/ros2.sh python3 scripts/inspect_pocket.py --help
```

抓包写入 `log/ground-link-capture.txt`。更换网卡后无需修改脚本默认接口。确认本机地址、实际UDP来源/目标和消息后，再修改配置。

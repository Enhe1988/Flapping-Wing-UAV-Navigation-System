# 通信与遥控诊断

当前链路为图像板192.168.1.151:9111 → 电脑192.168.1.100:9000，电脑控制消息发回192.168.1.151:9111。端口以通信launch参数和设备实际配置为准。

备用入口communication_backup.launch.py监听0.0.0.0:14550，发送到10.0.0.1:14555，关闭遥控采集与转发。用户截图确认10.0.0.100是电脑自己的地址，10.0.0.1是Backpack/AP网关。ID84默认发送10Hz；启动时传`velocity_rate_hz:=20.0`可改成20Hz，需重启通信生效。默认频率读取pocket_bench.yaml；输入指令仍必须在200ms有效期内，位置环继续以50Hz提供最新值。

## ELRS Backpack端口核对

Backpack官方默认配置的两个端口不同：

| 方向 | 目标地址与端口 |
| --- | --- |
| 电脑 → Backpack → 飞控 | Backpack实际IP:14555（Backpack的listen端口） |
| 飞控 → Backpack → 电脑 | 电脑实际IP:14550（Backpack的send端口） |

默认值见[Backpack配置源码](https://github.com/ExpressLRS/Backpack/blob/6883c5fef3dc234c3445e9b8809994d18308e8e8/lib/config/config.cpp#L55)。两个方向共用电脑绑定的UDP socket，并不要求两端监听端口相同。以前把备用发送端口也设为14550，不能按默认配置进入Backpack的上行监听器；已将备用launch的tx_port修正为14555，本地rx_port仍为14550。

[ELRS官方文档](https://www.expresslrs.org/software/mavlink/#setup-wifi-forwarding-for-mavlink)说明AP模式Backpack默认IP为10.0.0.1；用户截图也确认本机地址为10.0.0.100、默认网关为10.0.0.1。因此默认remote_ip已改为10.0.0.1。以前发往10.0.0.100实际会发给电脑自己，不能到达Backpack；UDP发送成功不代表目标是正确设备。

连接设备Wi-Fi后可在本机只读检查，联网与Codex会话不必同时可用：

```bash
ip -brief address
ip route
# 将地址替换成实际Backpack地址；此HTTP GET仅查询状态，不修改设备设置。
curl --noproxy '*' --max-time 3 http://10.0.0.1/mavlink
```

支持该接口的固件会返回ports.listen、ports.send、enabled和counters.packets_up。以实际端口值覆盖启动参数；只有packets_up随电脑发送增加，才能确认Backpack接收并解析了上行MAVLink，仍不能替代飞控收包确认。接口来源见[Backpack状态处理](https://github.com/ExpressLRS/Backpack/blob/6883c5fef3dc234c3445e9b8809994d18308e8e8/lib/WIFI/devWIFI.cpp#L570)。

官方AP默认配置的通信启动例子：

```bash
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication_backup.launch.py remote_ip:=10.0.0.1 tx_port:=14555 rx_port:=14550
```

若以后改成路由器STA模式，应按Backpack在该网络中的实际IP覆盖remote_ip。先停止旧通信进程后重启，检查启动行destination是否为10.0.0.1:14555。无需先启动位置环：通信节点本身每秒发心跳，没有位置指令时发送零XY的ID84，足够检查飞控是否收到消息。

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
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication.launch.py tx_enabled:=false use_rc:=false
# 备用链路只接收
bash scripts/ros2.sh ros2 launch flapping_uav_bringup communication_backup.launch.py tx_enabled:=false
```

诊断包含累计收发计数、配置的ID84发送频率velocity_rate_hz、发送错误、旧包/重复包丢弃及设备时钟重置计数。实际发送频率可用`ros2 topic hz /fc/velocity_setpoint_sent --window 40`观察。单条 `old/reordered sample dropped` 表示迟到包已被过滤，不等同于链路完全断开。

### 遥测时间戳异常与目标消失

当前样机ID250.time_usec约定为飞控uint32毫秒启动时钟乘1000，与ID31的time_boot_ms对应。桥接在更新顺序/新鲜度状态前拒绝超出该启动时钟范围的值，并限制设备时钟前进量不得比实际主机经过时间多出2秒。CRC通过只说明封包校验通过，不保证载荷时间字段合理；拒绝包不会刷新有效数据年龄或改写已接受的时钟基准。

`/diagnostics`中motion_invalid_timestamps记录不可能的ID250启动时间；motion_order_drops包含重复、迟到及不合理向前跳时包。控制器仍对真正的设备时间回退/姿态参考重置清除目标；不是简单关闭重启检测。

2026-11-25 04:51:32实验在t≈94.150s收到ID250.time_usec=6076276598041955504，前一正常值为117804000。旧校验把该异常大值当作新基准，随后将恢复正常的118606000误判为重启，t≈94.399s清除目标并删除RViz目标标记。同期ID31启动时钟连续递增，日志不支持真实重启。已新增该异常包和后续正常包的回环回归用例；异常帧产生于哪一环节仍需飞控/链路端进一步排查。

查看位置环真正发送的速度用 `/fc/velocity_setpoint_sent`，内部前正/右正指令用 `/fc/velocity_setpoint_body`。输入消息或时间戳停止更新，不能仅凭终端中最后一个非零数值判断仍在接收。

新日志的velocity_sent.csv记录前者的打包值、有效性和指令/姿态年龄，可区分“内部计算了非零指令”与“本机实际发送”。它仍不是飞控接收回执；核对ELRS转发和机上模式需要飞控接收计数/目标速度或代码。速度上限检查容许浮点舍入量并重新压回上限，避免2m/s限幅向量旋转后出现2+1e-15就被误发成零；真正超限、NaN、过期或无效指令仍输出零。

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

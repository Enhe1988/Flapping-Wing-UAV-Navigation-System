# 速度指令约4秒延迟的分段检查

2026-11-25实际速度测试中，用户观察的是飞控接收值`pc_data.vx_set_mps/vy_set_mps`，运行时自动刷新。电脑日志只能测本机发送，不能确认飞控到达时间；debug的实际刷新周期也要独立核对。

本次记录（本地时间）：

| 输入时间 | 速度(vx, vy)，m/s | 输入至首个匹配UDP发送监测 |
|---|---|---|
| 06:51:25.482 | (0.5, 0.5) | 75.56ms |
| 06:52:04.822 | (0.6, 0.6) | 36.92ms |
| 06:52:56.312 | (0.6, 0.6) | 47.25ms |
| 06:53:56.203 | (0.6, 0.6) | 58.26ms |
| 06:54:23.583 | (0.6, 0.6) | 78.44ms |

该会话UDP发送监测的最大间隔102.64ms。数据见`log/velocity_tests/20261125_065116_159036_velocity_test.csv`，分析结果见`log/velocity_tests/delay_analysis.json`，原始CSV未修改。这排除了ROS位置环及电脑程序主动等待4秒，但sendto成功仍不代表WiFi/ELRS已送达。

## 先核对接收与显示

保留一个通信节点和速度测试程序，停止导航并退出QGC/Mission Planner等可能额外发送MAVLink的程序。在地面测试中分别看`pc_data.velocity_rx_count`、`pc_data.velocity_update_ms`和接收速度，确认调试器是不中断CPU运行的持续刷新模式。

- 计数和接收时间本身数秒才批量刷新：还不能区分任务/链路延迟与调试显示刷新，需要下面的内部计时。
- 计数持续增长、但目标变化晚到：可能仍在消费排队的旧指令；计数增长不能证明数据新鲜。
- 接收值及时、实际采用值晚变：再查速度控制模式和摇杆仲裁。当前用户反馈为接收值晚变，优先查通信与解析。

## 飞控端无串口打印的计时

以下基于本地保存的飞控源码快照`7a1528f15d8c16d8e6b0c2b94513ab3abeb45afd`。代码需在实际飞控工程中人工合入并重新烧录；本ROS工作区没有修改或烧录飞控，也不能保证快照与板上版本完全相同。

该快照的`application/mavlink.c`在同一个任务中解析接收并发送遥测，发送使用阻塞式`HAL_UART_Transmit(..., 2000)`。2000ms是超时上限，**不是正常固定发送耗时**；只有实际发生超时/长阻塞时才可能拖慢下一次接收解析。先测量，不因4秒近似两次超时就断定原因。

可在该文件的全局区加入：

```c
volatile uint32_t mav_tx_last_ms = 0, mav_tx_max_ms = 0;
volatile uint32_t mav_tx_error_count = 0, mav_tx_last_status = 0;
volatile uint32_t mav_parse_last_ms = 0, mav_parse_max_gap_ms = 0;
volatile uint32_t mav_parse_call_count = 0;
```

将原发送函数替换为保留原行为、增加计时的版本（签名以实际工程为准）：

```c
void mavlink_send_uart_bytes(mavlink_channel_t chan,
                            const uint8_t *ch, int length)
{
    (void)chan;
    uint32_t begin = HAL_GetTick();
    HAL_StatusTypeDef result = HAL_UART_Transmit(
        &huart3, (uint8_t *)ch, length, 2000);
    uint32_t elapsed = HAL_GetTick() - begin;
    mav_tx_last_ms = elapsed;
    if (elapsed > mav_tx_max_ms) mav_tx_max_ms = elapsed;
    mav_tx_last_status = (uint32_t)result;
    if (result != HAL_OK) ++mav_tx_error_count;
}
```

在`Loop_Mavlink_Parse()`函数开头、原有处理之前加入：

```c
uint32_t now = HAL_GetTick();
if (mav_parse_call_count != 0U) {
    uint32_t gap = now - mav_parse_last_ms;
    if (gap > mav_parse_max_gap_ms) mav_parse_max_gap_ms = gap;
}
mav_parse_last_ms = now;
++mav_parse_call_count;
```

在debug中观察这些volatile计数。若发送最大耗时接近2000ms、错误次数增长，先修复UART发送状态/占用/配置，再考虑异步TX队列和独立RX解析；不可简单把超时改小后忽略丢帧。若解析调用最大间隔数秒而UART发送短，检查任务调度和其他阻塞。若两者一直很短，则优先沿ELRS转发链路或debug刷新排查。

## 遥控器转发统计

电脑连接Backpack WiFi后打开`http://10.0.0.1/`的MAVLink页，观察Uplink packets、Dropped packets和Buffer overflows在测试期间的增量。官方说明缓冲溢出代表数据量超过可用带宽；支持该统计的固件才会显示这些字段。[ExpressLRS官方说明](https://www.expresslrs.org/software/mavlink/#tx-backpack-mavlink-statistics)

2026-11-25用户截图：Send Port=14550、Listen Port=14555、GCS=10.0.0.100，符合当前ROS端口配置；Packets Downlink=40806、Packets Uplink=4642、Drops Downlink=42916、Overflows Downlink=0。当前官方Backpack源码用一个expectedSeq跟踪全部下行消息，没有按sysid/compid分别计数，因此多来源交错、乱序、重启也可能放大序号缺口。不能由累计Drops计算可靠RF丢包率，也不能据此判定上行速度指令丢失。

源码中Packets Uplink在Backpack解析电脑消息并调用Serial.write后增加，代表Backpack已经接收并交给本地串口，不是飞控接收确认；下行溢出为0也不证明上行无排队。仅运行当前10Hz速度发送与1Hz心跳时，10秒Uplink增量应约110；应核对短窗口增量和响应时刻，而非只看累计值。[Backpack统计实现](https://github.com/ExpressLRS/Backpack/blob/master/lib/MAVLink/MAVLink.cpp)

当前F1000、10Hz ID84的标称数据量未必超出链路能力；实际重传、额外发送者和固件版本仍可能影响可用吞吐，不能仅凭几秒延迟就认定是带宽。若有缓冲溢出，先消除额外发送者，并核对TX/RX/Backpack版本和链路质量。若做降低发送频率的对照，只用于地面诊断，需同时考虑飞控300ms目标有效期，不能把过低频率当作正常闭环配置。

若上述计时均正常而仍有约4秒接收延迟，最直接的下一项测量是在接收机TX→飞控RX处抓串口并解码ID84，比较线上的指令变化和飞控解析更新；这样可以区分无线转发排队与飞控内部处理。

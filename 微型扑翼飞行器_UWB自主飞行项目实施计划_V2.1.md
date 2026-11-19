---
date: 2026-11-06
subtitle: 完整项目实施计划与接口规范
title: 微型扑翼飞行器 UWB 自主导航与轨迹飞行系统
version: V2.1
---

# 微型扑翼飞行器 UWB 自主导航与轨迹飞行系统

## 完整项目实施计划与接口规范 V2.1

**用途：** 交给 Ubuntu 环境中的 Codex 阅读、审查并分阶段实施。\
**开发环境：** Ubuntu、ROS 2、C++。\
**飞控：** 自研飞控，原有遥控器协议为 CRSF（ExpressLRS）。\
**全局定位：** Nooploop LinkTrack UWB。\
**机载辅助传感器：** 光流（已解算机体速度）及测距模块。\
**遥控器：** RadioMaster Pocket（EdgeTX，USB HID Joystick）。\
**通信：** MAVLink 2、UDP、UART 图传透传。

------------------------------------------------------------------------

# 0. 本文档的使用规则

本文档是本项目当前版本的正式需求基线。Codex
应先审查已有工程，再按阶段实施。

**必须遵守的最终架构决定：**

1.  地面站始终发送三类数据：`HEARTBEAT (ID 0)`、`RC_CHANNELS_OVERRIDE (ID 70)`、`SET_POSITION_TARGET_LOCAL_NED (ID 84)`。
2.  地面站不区分 MANUAL/AUTO，不建立地面站 Control
    Mux，也不根据飞行模式停止其中一条数据流。
3.  飞控自己进行飞行模式选择：在其内部决定使用 RC 通道输入还是外部 XYZ
    速度设定值。
4.  飞控始终发送两类数据：`HEARTBEAT (ID 0)` 和 `ODOMETRY (ID 331)`。
5.  `ODOMETRY` 用于回传光流已经解算的机体速度、有效的高度 Z 和姿态。
6.  UWB 数据直接进入地面电脑 ROS 2；不把 UWB XY 发送给飞控。
7.  飞控原有 CRSF 通道处理、速度环和姿态环尽可能保留。新增 MAVLink
    输入适配层，不重写现有控制算法。
8.  第一阶段实现平整、受控场地内的 XY
    点到点飞行；高度先由飞控机载传感器和现有高度控制能力保持。暂不实现障碍物避障。

**不要擅自更换消息：** 本项目明确使用
`RC_CHANNELS_OVERRIDE (70)`，不改用 `RADIO_RC_CHANNELS (420)`。

------------------------------------------------------------------------

# 1. 项目目标与总体架构

## 1.1 最终目标

用户在 RViz 中点击目标位置，地面站根据 UWB
实时位置计算期望速度，经图传发送给自研飞控。飞控根据自己的模式状态决定是否使用外部速度设定值，并由机载速度环和姿态环控制扑翼飞行器移动至目标位置。

最终应实现：

-   UWB 实时 XY 定位；
-   RViz 显示飞行器当前位置和运动轨迹；
-   RViz 点击目标点；
-   ROS 2 地面位置环计算期望 XYZ 速度；
-   MAVLink 将期望速度送入飞控；
-   飞控内部自行切换手动/自主模式；
-   飞控复用原有速度环、姿态环；
-   Pocket 通过 USB 接入 Ubuntu，通道数据经 MAVLink 70 发送给飞控；
-   飞控回传光流机体速度、高度和姿态；
-   到达目标后平稳减速和停止。

## 1.2 确认的硬件和系统条件

  项目                      已知情况
  ------------------------- --------------------------------------
  飞行器                    微型扑翼飞行器
  飞控                      自研飞控
  飞控原有 RC 协议          CRSF（ELRS）
  飞控机载状态              IMU 姿态、光流解算机体速度、测距高度
  UWB                       Nooploop LinkTrack
  地面电脑连接的 UWB 设备   Anchor 基站
  UWB 连接方式              USB 转 TTL
  飞行器 UWB 设备           Tag，具体 ID 待确认
  图传                      天空端 + 地面端，UART/UDP 透传
  图传地面端 IP             `192.168.1.100`
  地面站                    Ubuntu 电脑
  地面站开发                ROS 2 C++
  遥控器                    RadioMaster Pocket
  遥控器接入                USB HID Joystick

## 1.3 系统分工

**地面电脑 ROS 2：** - LinkTrack UWB 数据接收、解析和质量检查； - UWB XY
全局位置处理； - 位置环和轨迹生成； - RViz 目标点交互； - Pocket USB HID
数据读取； - MAVLink 双向 UDP 通信； - 记录和显示飞控状态。

**自研飞控：** - IMU 姿态估计； - 光流机体速度估计； -
测距和高度估计； - 速度环 PID； - 姿态环 PID； - 扑翼机构控制； - 内部
MANUAL/AUTO 模式管理； - MAVLink 数据接收、发送和超时保护； - CRSF 与
MAVLink RC 输入适配。

## 1.4 总体数据流

``` text
UWB Anchor --USB-TTL--> nlink_parser2 --> uwb_position_node
                                              |
                                              v
                                         UWB XY / map
                                              |
RViz 2D Nav Goal --> goal_manager --> position_controller
                                              |
                                              v
                                  期望世界坐标系 XYZ 速度
                                              |
                                              v
                                  转换为 LOCAL_NED 速度
                                              |
                                              v
Pocket --> USB HID --> joy_node --> manual_control_node
                                              |
                                              v
                               RC_CHANNELS_OVERRIDE (70)
                                              |
                 +----------------------------+
                 |
                 v
          mavlink_bridge_node
                 |
        UDP -> 图传地面端 192.168.1.100:16000
                 |
             图传无线链路
                 |
              自研飞控
                 |
       飞控内部模式选择（由飞控负责）
          /                    \
    RC 通道输入             外部速度设定值
          \                    /
            原有控制系统 / 速度环
                       |
                    姿态环
                       |
                  扑翼执行机构

飞控 -> ODOMETRY (331)：BODY_FRD 机体速度 + LOCAL_NED 高度/姿态
飞控 -> HEARTBEAT (0)
经图传回传至 Ubuntu 本地 UDP 14500。
```

地面站两条控制数据流相互独立并始终发送： - Pocket 通道流； - 期望 XYZ
速度流。

**地面站不做 MANUAL/AUTO 控制源仲裁。飞控自行选择当前模式对应的输入。**

------------------------------------------------------------------------

# 2. 飞控与地面站 MAVLink 接口规范

本章是两端开发的共同接口合同。字段、单位、坐标系和消息语义必须一致。

## 2.1 协议版本

-   全链路使用 MAVLink 2。
-   使用 MAVLink Common 标准消息。
-   飞控与地面站使用兼容的消息定义和生成代码。
-   不得私自修改标准消息字段。
-   自定义 MAVLink
    消息只有在标准消息无法表达必要数据时才考虑；当前首版回传优先使用
    `ODOMETRY`。

官方消息定义： - https://mavlink.io/en/messages/common.html -
https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE -
https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED -
https://mavlink.io/en/messages/common.html#ODOMETRY

## 2.2 最终消息清单

  -------------------------------------------------------------------------------------------------------
  方向         消息                                            ID     初始建议频率 用途
  ------------ --------------------------------- ---------------- ---------------- ----------------------
  地面站 →     `HEARTBEAT`                                      0             1 Hz 地面站在线状态
  飞控

  地面站 →     `RC_CHANNELS_OVERRIDE`                          70        30--50 Hz Pocket 摇杆和开关通道
  飞控

  地面站 →     `SET_POSITION_TARGET_LOCAL_NED`                 84        20--50 Hz 期望 XYZ 速度
  飞控

  飞控 →       `HEARTBEAT`                                      0             1 Hz 飞控在线和状态
  地面站

  飞控 →       `ODOMETRY`                                     331        20--50 Hz 机体速度、高度、姿态
  地面站
  -------------------------------------------------------------------------------------------------------

频率为初始工程建议，不是硬性标准。应结合 UWB
更新率、飞控任务频率、图传吞吐量和实测丢包情况调整。

## 2.3 地面站 → 飞控：HEARTBEAT（ID 0）

方向：GCS → FC。\
建议频率：1 Hz。

作用：告知飞控地面站 MAVLink 系统在线。

建议字段： - `type`：`MAV_TYPE_GCS`； -
`autopilot`：`MAV_AUTOPILOT_INVALID`； -
`base_mode`、`custom_mode`：按地面站身份填写； -
`system_status`：按实际地面站状态填写； - `mavlink_version`：由 MAVLink
库按规范填写。

注意：HEARTBEAT 在线不代表 RC 数据或速度设定值新鲜。飞控必须分别监控
RC_OVERRIDE 和速度设定值的最后接收时间。

## 2.4 地面站 → 飞控：RC_CHANNELS_OVERRIDE（ID 70）

方向：GCS → FC。\
建议频率：30--50 Hz。\
作用：传输 RadioMaster Pocket 的摇杆、开关和通道数据。

主要字段：

  字段                           类型         单位
  ------------------------------ ------------ ------------------------
  `target_system`                `uint8_t`    ---
  `target_component`             `uint8_t`    ---
  `chan1_raw` ... `chan18_raw`   `uint16_t`   通道值，通常按 μs 表示

通道数值示例（必须以现有飞控 RC 标定为准）：

  物理状态     示例值
  ---------- --------
  最小           1000
  中位           1500
  最大           2000

MAVLink 标准忽略/释放语义必须严格实现：

-   CH1--CH8：`UINT16_MAX` 表示忽略该通道；`0` 表示释放该通道覆盖。
-   CH9--CH18：`0` 或 `UINT16_MAX` 表示忽略；`UINT16_MAX-1`
    表示释放该通道覆盖。
-   `1500` 是典型摇杆中位值，不是释放值。
-   不得把 `0` 当作摇杆中位值。
-   CH9--CH18 是 MAVLink 2 扩展字段，确保使用 MAVLink 2。

Pocket 读取后应将实际 HID
轴和按钮转换为飞控当前使用的通道顺序和范围。不得擅自改变飞控已有通道定义。

飞控收到 Message 70 后，应转换为与现有 CRSF
解码器后级内部通道表示一致的数据，再进入已有 RC 通道处理逻辑。

如果飞控同时存在实体 CRSF 输入和 MAVLink 虚拟 RC
输入，飞控端必须定义明确的输入源优先级/有效性策略，不能让两个解析器无保护地竞争写入同一通道缓存。该策略属于飞控输入适配，不是地面站
MANUAL/AUTO Control Mux。

## 2.5 地面站 → 飞控：SET_POSITION_TARGET_LOCAL_NED（ID 84）

方向：GCS → FC。\
建议频率：20--50 Hz。\
作用：传输位置控制器计算的期望 XYZ 速度。

消息字段：

  字段                 类型         单位    说明
  -------------------- ------------ ------- ------------------------
  `time_boot_ms`       `uint32_t`   ms      地面站本地启动后时间
  `target_system`      `uint8_t`    ---     飞控 System ID
  `target_component`   `uint8_t`    ---     飞控 Component ID
  `coordinate_frame`   `uint8_t`    ---     参考坐标系
  `type_mask`          `uint16_t`   ---     字段有效性掩码
  `x,y,z`              `float`      m       位置字段，本项目忽略
  `vx,vy,vz`           `float`      m/s     期望速度
  `afx,afy,afz`        `float`      m/s²    加速度字段，本项目忽略
  `yaw`                `float`      rad     本项目忽略
  `yaw_rate`           `float`      rad/s   本项目忽略

### 坐标系

``` text
coordinate_frame = MAV_FRAME_LOCAL_NED
```

定义： - X：North，北向为正； - Y：East，东向为正； -
Z：Down，向下为正。

因此： - `vx`：期望北向速度； - `vy`：期望东向速度； -
`vz`：期望向下速度。

### type_mask

本项目只使用 VX、VY、VZ，忽略位置、加速度、Yaw、Yaw Rate。

``` text
type_mask = 3527
```

计算依据：

``` text
POSITION_IGNORE = 1 + 2 + 4
ACCEL_IGNORE    = 64 + 128 + 256
YAW_IGNORE      = 1024
YAW_RATE_IGNORE = 2048

type_mask = 7 + 448 + 1024 + 2048
           = 3527
```

不得置位 VX_IGNORE、VY_IGNORE、VZ_IGNORE。不得置位 FORCE_SET。

Codex 必须在所使用的 MAVLink
生成头文件中核对掩码常量，避免位定义或版本差异。

### 第一阶段的 Z 轴

第一阶段只做 XY 点到点飞行，地面站暂时发送：

``` text
vz = 0.0 m/s
```

其含义是期望垂直速度为零，并不自动等于高度保持。Codex
必须检查飞控是否已有可用的垂直速度/高度保持控制逻辑。如果现有飞控没有对应闭环，应先报告，不得假设发送零速度就能保持高度。

### 时间戳和超时

第一阶段不要求地面站与飞控精确时钟同步。`time_boot_ms`
可使用地面站单调计时器计算的毫秒数。飞控的超时保护必须依据飞控本地接收时刻，而不是依赖两端时钟同步。

飞控必须检查： - `target_system`、`target_component`； -
`coordinate_frame` 是否为支持的 LOCAL_NED； - `type_mask`
是否符合速度模式； - VX/VY/VZ 是否为有限数值； -
速度是否超过安全限幅； - 最近一次有效速度设定值的接收时间。

## 2.6 飞控 → 地面站：HEARTBEAT（ID 0）

方向：FC → GCS。\
建议频率：1 Hz。

用途： - 飞控在线检测； - 解锁/安全状态； - 飞行模式状态； -
飞控系统状态。

地面站应显示飞控在线、离线和状态变化。

HEARTBEAT 不代替 ODOMETRY 数据有效性检查。

## 2.7 飞控 → 地面站：ODOMETRY（ID 331）

方向：FC → GCS。\
建议频率：20--50 Hz。

用途：统一回传： - 光流已经解算的机体速度； - 飞控本地高度 Z； -
姿态四元数； - 可用的角速度； - 估计质量和协方差信息。

选择 ODOMETRY
的原因：该消息允许位置参考系和速度参考系分别指定，适合本项目"高度/姿态在
LOCAL_NED 参考下，速度在 BODY_FRD 机体坐标系下"的数据组织。

### 坐标配置

``` text
frame_id       = MAV_FRAME_LOCAL_NED
child_frame_id = MAV_FRAME_BODY_FRD
```

语义： - `frame_id`：位置和姿态参考坐标系； -
`child_frame_id`：速度参考坐标系。

### 字段使用

  字段                    类型          单位    本项目用途
  ----------------------- ------------- ------- ------------------------
  `time_usec`             `uint64_t`    μs      飞控本地时间戳
  `frame_id`              `uint8_t`     ---     LOCAL_NED
  `child_frame_id`        `uint8_t`     ---     BODY_FRD
  `x,y`                   `float`       m       飞控无全局 XY 时不可用
  `z`                     `float`       m       LOCAL_NED 高度坐标
  `q[4]`                  `float[4]`    ---     姿态四元数，WXYZ
  `vx`                    `float`       m/s     机体前向速度
  `vy`                    `float`       m/s     机体右向速度
  `vz`                    `float`       m/s     机体向下速度
  `rollspeed`             `float`       rad/s   横滚角速度
  `pitchspeed`            `float`       rad/s   俯仰角速度
  `yawspeed`              `float`       rad/s   偏航角速度
  `pose_covariance`       `float[21]`   ---     位姿协方差上三角
  `velocity_covariance`   `float[21]`   ---     速度协方差上三角
  `reset_counter`         `uint8_t`     ---     估计重置计数
  `estimator_type`        `uint8_t`     ---     状态估计器类型
  `quality`               `int8_t`      ---     估计质量

### BODY_FRD 速度定义

-   X：机头向前为正；
-   Y：机体向右为正；
-   Z：机体向下为正。

`vx`、`vy`
必须是光流模块已经解算的机体速度，不能发送原始光流像素流或未经比例/姿态补偿的中间量。

Codex 必须通过飞控代码和实测确认光流速度的轴向、正负号、单位和安装方向。

### LOCAL_NED 高度定义

假设本地 NED 原点位于起飞地面，且高度经过必要的姿态补偿：

``` text
ODOMETRY.z = -height_agl
```

例如离地高度 1 m，则 `z = -1 m`。

如果飞控 LOCAL_NED 原点并非起飞地面，必须使用经过标定的 Z
偏置，不能直接假设 `z = -height_agl`。

地面站计算离地高度：

``` text
height_agl = -ODOMETRY.z
```

如果高度无效，应按 ODOMETRY 规范标记不可用，不得发送伪造的零高度。

### 水平位置

飞控没有 UWB 全局 XY 时： - `x`、`y` 不得伪装成有效全局位置； -
使用协议允许的 NaN 表示未知； - 地面站不得把 ODOMETRY.x/y 当作 UWB
全局位置。

### 姿态

四元数数组顺序： - q\[0\] = W； - q\[1\] = X； - q\[2\] = Y； - q\[3\] =
Z。

必须按照 MAVLink ODOMETRY
对姿态的定义填充，不得自行交换顺序或反转旋转方向。地面站应使用该四元数把
BODY_FRD 速度转换到相应世界参考系。

### 协方差和质量

-   `pose_covariance` 和 `velocity_covariance` 按 MAVLink 规定的 21
    元素上三角布局填写；
-   未知协方差按协议规定使用 NaN，不要用 0 冒充"完全准确"；
-   `quality` 按 ODOMETRY 规范使用 -1 表示估计失败、0
    表示未知/未设置、1--100 表示质量值；
-   如果光流质量原始范围不是 0--100，必须先确认映射方法；
-   `reset_counter`、`estimator_type` 按实际估计器填写。

### ODOMETRY 有效性

地面站应分别判断： - 速度有效性； - 高度有效性； - 姿态有效性； -
数据新鲜度； - 质量是否满足控制要求。

不能因为收到 ODOMETRY 就认定所有字段都有效。

------------------------------------------------------------------------

# 3. UDP 图传通信

## 3.1 已确认网络结构

``` text
自研飞控
   |
UART / MAVLink 2
   |
图传天空端
   |
无线透传
   |
图传地面端
IP: 192.168.1.100
   |
Ethernet
   |
Ubuntu 地面电脑
```

## 3.2 UDP 参数

  方向                  目标地址                  端口 协议
  --------------------- -------------------- --------- ------
  Ubuntu → 图传地面端   `192.168.1.100`        `16000` UDP
  图传地面端 → Ubuntu   Ubuntu 电脑自身 IP     `14500` UDP

Ubuntu： - 发送目标：`192.168.1.100:16000`； -
接收绑定：`0.0.0.0:14500`。

图传地面端把天空端串口收到的 MAVLink 数据转发到 Ubuntu 电脑实际 IP 的
14500 端口。

`192.168.1.100` 是图传地面端 IP，不是 Ubuntu 电脑 IP。

## 3.3 MAVLink Bridge

ROS 2 节点：

`mavlink_bridge_node`

职责： 1. 创建 UDP 接收 Socket，绑定 14500； 2. 创建 UDP 发送
Socket，发送到 192.168.1.100:16000； 3. 解析 MAVLink 2 数据帧； 4. 接收
HEARTBEAT 和 ODOMETRY； 5. 发布飞控状态 Topic； 6. 订阅 RC
通道数据并封装 Message 70； 7. 订阅期望 NED 速度并封装 Message 84； 8.
周期发送地面站 HEARTBEAT； 9. 记录消息频率、丢包、解析错误和超时状态。

通信层只负责协议和网络，不实现位置 PID。

------------------------------------------------------------------------

# 4. LinkTrack UWB 接入 ROS 2

## 4.1 硬件连接

已知： - 地面电脑连接 LinkTrack Anchor； - 使用 USB 转 TTL； -
飞行器携带 Tag； - 多个 Anchor 构成定位系统。

当前尚未确认： - Anchor 的实际输出协议； - Tag ID； - 串口设备名称； -
实际波特率； - UWB 更新频率。

## 4.2 官方 ROS 2 驱动

使用 Nooploop 官方 ROS 2 驱动：

`nlink_parser2`

仓库： https://github.com/nooploop-dev/nlink_parser2

不使用 ROS 1，不增加 ROS 1/ROS 2 桥接。

驱动通过串口解析 LinkTrack 协议并发布 ROS 2 消息。

921600 是常见示例波特率，必须与 NAssistant
和设备实际设置一致，不得直接硬编码为已确认值。

## 4.3 输出协议确认

优先检查 NAssistant 中 Anchor 的输出协议。

`Anchor_Frame0` 是当前候选方案，不是已经确认的配置。

如果实际使用 Anchor_Frame0，候选话题为：

`/nlink_linktrack_anchorframe0`

候选消息类型：

`nlink_parser2/msg/LinktrackAnchorframe0`

从节点数组中按实际 Tag ID 筛选飞行器位置。

如果当前输出协议不同，先根据官方驱动的真实话题和消息定义确定订阅接口。

不得假设： - 飞行器一定是 T0； - `nodes[0]` 一定是飞行器； -
Anchor_Frame0 已经启用； - 所有 Anchor/Tag 协议输出都相同。

## 4.4 uwb_position_node

建议节点名称：

`uwb_position_node`

职责： - 订阅官方 nlink_parser2 话题； - 根据 Tag ID 筛选目标； - 提取
XYZ； - 检查 Tag 是否存在； - 检查数据是否过期； -
检查坐标跳变和异常值； - 根据实测结果滤波； - 完成 UWB 到 ROS map
的坐标转换； - 发布统一位置 Topic。

输出 Topic：

`/uwb/position`

建议类型：

`geometry_msgs/msg/PoseWithCovarianceStamped`

坐标系： - `header.frame_id = map`； - x、y 为 map 坐标； - 第一阶段 z
只记录，不参与位置控制； - 协方差应根据实际静态定位数据估计。

UWB 位置失效或过期时，必须显式标记无效，不能一直把旧位置当作新数据使用。

------------------------------------------------------------------------

# 5. RadioMaster Pocket USB 遥控器

## 5.1 数据链路

``` text
RadioMaster Pocket
      |
USB-C 数据线
      |
Ubuntu USB HID
      |
ROS 2 joy_node
      |
sensor_msgs/msg/Joy
      |
manual_control_node
      |
RC 通道映射
      |
MAVLink RC_CHANNELS_OVERRIDE (70)
      |
图传 -> 飞控
```

本方案使用 USB HID，不使用 Pocket 自身 ELRS 无线发射链路作为地面站输入。

## 5.2 EdgeTX 配置原则

-   建立独立 USB 控制模型；
-   USB 模式使用 Joystick；
-   优先测试 Classic Joystick；
-   根据实际 HID 测试确认轴、按钮和开关；
-   不修改原有飞行模型；
-   不要预设摇杆模式、轴索引或通道顺序。

## 5.3 ROS 2 接入

使用 ROS 2 `joy` 软件包。

节点： - `joy_node`

原始 Topic： - `/joy`

消息： - `sensor_msgs/msg/Joy`

新增节点： - `manual_control_node`

负责： - 轴索引映射； - 方向校准； - 中位校准； - 死区； - 归一化； -
摇杆数值转换为飞控 RC 通道范围； - 开关和按钮映射； - USB
设备断开检测； - 生成完整 RC 通道数据。

必须实际测试 Pocket 的 USB HID 轴和按钮顺序，不得假定所有 EdgeTX 版本或
USB 模式输出相同。

如果 USB 断开，不得无限期重复发送最后一组非零摇杆值。按 RC_OVERRIDE
释放语义和飞控超时保护设计安全处理。

------------------------------------------------------------------------

# 6. 自研飞控改造

## 6.1 保留现有 CRSF 逻辑

飞控现有遥控器协议为 CRSF（ELRS）。

现有处理链路：

``` text
CRSF 接收机
    |
CRSF 解码器
    |
内部 RC 通道结构
    |
原有通道处理 / 模式管理
    |
速度环
    |
姿态环
    |
执行机构
```

原则： - 不重写 CRSF 解码器； - 不重写原有通道处理逻辑； -
不重写速度环； - 不重写姿态环。

## 6.2 增加 MAVLink RC Adapter

新增适配层：

`MAVLink RC Adapter`

输入： - `RC_CHANNELS_OVERRIDE (70)`。

工作： 1. 解析 Message 70； 2. 检查目标系统和组件； 3.
按标准处理有效、忽略和释放通道； 4. 转换为原有 CRSF
解码器后级的内部通道表示； 5. 输出到统一 RC 输入接口。

Codex 必须先查清： - CRSF 原始 11 位通道范围； - CRSF
解码后的内部类型； - 原有通道归一化方式； - 通道中位； - 通道顺序； -
通道有效性标志； - 现有模式开关对应的通道。

不得假设原有内部结构一定存储 1000--2000 μs。

如果 CRSF 与 MAVLink
两个输入源可能同时存在，飞控端要定义明确优先级或来源选择，避免两个数据源竞争写同一个通道数组。

## 6.3 增加外部速度设定值接收

新增：

`MAVLink Velocity Setpoint Adapter`

接收：

`SET_POSITION_TARGET_LOCAL_NED (84)`。

提取： - VX； - VY； - VZ； - coordinate_frame； - type_mask； -
接收时间； - 目标系统和组件。

检查： - 是否为 LOCAL_NED； - 是否为本项目速度-only mask； -
速度是否为有限值； - 是否超出限幅； - 数据是否过期。

飞控内部模式管理自行决定是否使用外部速度设定值。

**地面站不根据模式切换停止发送任何一路数据。**

## 6.4 NED 到 BODY_FRD 转换

地面站发送的期望速度是 LOCAL_NED 世界坐标系速度。

光流反馈速度是 BODY_FRD 机体坐标系速度。

飞控速度环如果使用机体坐标系速度，必须根据当前姿态将期望 NED 速度转换为
BODY_FRD，再进入现有速度环。

不得直接将 NED 期望速度与 BODY_FRD 实际速度比较。

优先复用飞控已有姿态四元数和坐标转换模块。

------------------------------------------------------------------------

# 7. ROS 2 工程结构

使用 C++ / rclcpp。

建议工作空间：

``` text
flapping_uav_ws/
└── src/
    ├── nlink_parser2/
    ├── flapping_uav_msgs/
    ├── flapping_uav_gcs/
    └── flapping_uav_bringup/
```

`nlink_parser2` 保持官方驱动独立，不直接修改官方源码来加入项目控制逻辑。

## 7.1 节点清单

  节点                         功能
  ---------------------------- -------------------------
  `nlink_parser2`              官方 UWB 串口驱动
  `uwb_position_node`          UWB Tag 筛选、位置处理
  `mavlink_bridge_node`        MAVLink 双向 UDP
  `joy_node`                   Pocket USB HID
  `manual_control_node`        摇杆/开关转换为 RC 通道
  `position_controller_node`   UWB XY 位置环
  `goal_manager_node`          RViz 目标点和任务状态
  `rviz2`                      可视化

**不建立 `control_mux_node`。**

地面站不实现 MANUAL/AUTO 模式仲裁。`manual_control_node` 与
`position_controller_node` 分别输出各自数据流，MAVLink Bridge
同时发送两种消息。

## 7.2 ROS 2 Topic 规划

  -----------------------------------------------------------------------------------------------------
  Topic                         类型                                            用途
  ----------------------------- ----------------------------------------------- -----------------------
  `/uwb/position`               `geometry_msgs/msg/PoseWithCovarianceStamped`   UWB 全局位置

  `/fc/odometry`                `nav_msgs/msg/Odometry`                         飞控速度、高度和姿态

  `/fc/heartbeat`               项目状态封装或状态 Topic                        飞控在线状态

  `/joy`                        `sensor_msgs/msg/Joy`                           Pocket 原始输入

  `/rc/override`                项目自定义 RC 通道消息                          发送 Message 70 的数据

  `/fc/velocity_setpoint_ned`   `geometry_msgs/msg/TwistStamped`                期望 NED XYZ 速度

  `/goal_pose`                  `geometry_msgs/msg/PoseStamped`                 RViz 目标点

  `/navigation/goal`            `geometry_msgs/msg/PoseStamped`                 当前任务目标
  -----------------------------------------------------------------------------------------------------

消息适配要求： - `/fc/odometry` 的 `header.frame_id` 和 `child_frame_id`
必须反映真实坐标系； - BODY_FRD 速度不能标记成 map 速度； -
速度坐标变换必须显式执行； - 不得仅为了符合 ROS 消息结构而改变 MAVLink
原始语义。

------------------------------------------------------------------------

# 8. 坐标系与控制算法

## 8.1 坐标系

至少管理： - UWB 原始坐标系； - ROS `map`； - ENU； - MAVLink
LOCAL_NED； - MAVLink BODY_FRD。

ROS 内部建议使用 ENU 语义。

ENU： - X：East； - Y：North； - Z：Up。

NED： - X：North； - Y：East； - Z：Down。

当 map 与 ENU 完全对齐时：

``` text
NED_X = ENU_Y
NED_Y = ENU_X
NED_Z = -ENU_Z
```

如果 UWB 坐标系与 ENU
存在旋转偏差，必须通过现场标定求得固定变换，不得直接假定 UWB X/Y 就等于
East/North。

## 8.2 位置环

节点：

`position_controller_node`

输入： - `/uwb/position`； - `/goal_pose` 或 `/navigation/goal`； -
`/fc/odometry`。

输出： - `/fc/velocity_setpoint_ned`。

第一版采用：

位置 P/PD 控制 + 速度限幅 + 加速度限幅。

基本位置误差：

``` text
ex = x_goal - x_current
ey = y_goal - y_current
```

先在 map/ENU 中计算期望速度：

``` text
v_map_des = Kp * position_error
```

可增加速度反馈阻尼：

``` text
v_map_des = Kp * position_error - Kd * v_map_actual
```

其中 `v_map_actual` 必须由 ODOMETRY 的 BODY_FRD
实际速度结合姿态四元数转换到 map/ENU 后获得。

再执行： 1. map 到 ENU 的固定标定变换； 2. ENU 到 NED 的坐标转换； 3.
限制最大水平速度； 4. 限制速度变化率； 5. 发布 NED 速度设定值； 6.
MAVLink Bridge 封装 Message 84。

参数不得直接拍脑袋确定。先进行低速、短距离、受控测试，再逐步调节。

## 8.3 到达判定

不只根据位置误差判断到达。

建议同时检查： - 水平位置误差低于阈值； - 实际水平速度低于阈值； -
条件连续保持一段时间。

满足后输出零水平速度设定值，避免飞机以较高速度穿过目标点。

## 8.4 第一阶段 Z 轴

第一阶段不使用 UWB Z 参与控制。

-   飞控回传机载高度；
-   飞控自行执行已有高度保持；
-   地面站 `vz=0`；
-   如果飞控没有垂直速度/高度闭环，先补充接口审查和方案设计，不能假定零速度自动等于高度保持。

------------------------------------------------------------------------

# 9. RViz 目标点交互

RViz： - Fixed Frame：`map`； - 使用 `2D Nav Goal`； - 发布
`/goal_pose`； - 消息类型 `geometry_msgs/msg/PoseStamped`。

执行过程： 1. RViz 显示 UWB 实时位置； 2. 用户点击目标点； 3. Goal
Manager 保存目标； 4. Position Controller 计算期望速度； 5. MAVLink
Bridge 持续发送 Message 84； 6. 飞控内部模式选择是否执行； 7.
飞行器向目标点移动； 8. 到达条件满足后发送零水平速度； 9.
用户可以通过飞控现有模式切换进行人工接管。

第一阶段忽略目标 Yaw，保持当前航向。

不引入 Nav2，不实现复杂避障。实验场地应为平整、清空的受控区域。

------------------------------------------------------------------------

# 10. 安全保护与异常处理

## 10.1 地面站

-   UWB 数据过期：不得继续用旧位置计算有效新控制指令；
-   UWB 无效：禁止启动新的自主任务；
-   ODOMETRY 过期：停止依赖旧速度反馈，按预定策略处理；
-   姿态无效：禁止继续使用依赖姿态变换的速度反馈；
-   Pocket USB 断开：标记 RC 输入无效，按释放/超时机制处理；
-   图传中断：显示飞控离线；
-   目标取消：输出零速度设定值；
-   所有期望速度进行限幅和有限数值检查。

## 10.2 飞控

必须独立记录： 1. 地面站 HEARTBEAT 最后接收时间； 2.
RC_CHANNELS_OVERRIDE 最后有效接收时间； 3. SET_POSITION_TARGET_LOCAL_NED
最后有效接收时间。

三者不可互相替代。

初始工程建议： - RC_OVERRIDE 超时约 300 ms； - 速度设定值超时约 300
ms； - HEARTBEAT 超时可先按约 3 s 设计。

以上是初始建议值，必须通过实际图传丢包和延迟测试调整。

超时后： - 不得无限保持最后一条非零速度指令； -
飞控根据当前模式和机载状态执行本地安全策略； -
若高度估计有效，优先保证高度安全； -
若光流、姿态或高度状态失效，进入飞控本地定义的安全状态。

紧急停止不得只依赖普通 UDP 控制消息。

## 10.3 模式切换

模式切换完全由飞控内部实现。

-   MANUAL 模式：飞控按原有 RC 逻辑处理通道。
-   AUTO 模式：飞控使用外部 SET_POSITION_TARGET_LOCAL_NED 速度设定值。
-   地面站始终发送 RC 通道和速度设定值。
-   飞控必须保证两类输入不会同时直接作用于速度环。
-   RC 模式开关、安全开关等按飞控现有逻辑处理。

------------------------------------------------------------------------

# 11. 分阶段实施与验收

## 阶段 0：现有工程审查

先检查，不大规模修改代码。

### Ubuntu 检查

-   Ubuntu 版本；
-   ROS 2 发行版；
-   C++ 编译工具；
-   colcon；
-   MAVLink 库；
-   当前工作空间和已有包。

### 飞控检查

-   工程目录；
-   MAVLink 版本和生成代码；
-   UART 通信；
-   Message 70 当前实现；
-   Message 84 接收能力；
-   Message 331 发送能力；
-   CRSF 解码器；
-   CRSF 解码后内部 RC 结构；
-   RC 通道映射；
-   飞行模式管理；
-   速度环输入；
-   姿态四元数；
-   光流速度变量；
-   高度变量；
-   NED/BODY_FRD 坐标变换。

### 阶段 0 交付物

-   工程审查报告；
-   现有代码模块图；
-   关键文件清单；
-   MAVLink 消息支持情况；
-   需要修改的最小文件范围；
-   坐标系和单位确认表；
-   风险清单。

完成后等待用户确认，再开始阶段 1。

## 阶段 1：UWB 独立接入

任务： - 安装官方 nlink_parser2； - 确认 Anchor 串口和波特率； - 确认
NAssistant 输出协议； - 确认 Tag ID； - 订阅真实定位 Topic； - 开发
uwb_position_node； - RViz 显示实时 XY。

验收： - 正确筛选飞行器 Tag； - 能稳定获取 XY； - 能检测数据失效； -
能记录实际更新频率和静态噪声； - RViz 位置显示正确。

## 阶段 2：MAVLink 遥测接收

任务： - UDP 14500 接收； - MAVLink 2 解析； - HEARTBEAT 接收； -
ODOMETRY 接收； - 转换为 ROS 2 状态 Topic； - 显示机体速度、高度和姿态。

验收： - 飞控在线状态正常； - 速度正负方向正确； -
高度符号和单位正确； - 姿态四元数正确； - 数据频率和丢包有记录； -
暂不发送实际运动指令。

## 阶段 3：Pocket USB 接入

任务： - EdgeTX USB Joystick； - Ubuntu HID 识别； - joy_node； -
轴和按钮映射； - RC 通道转换； - Message 70 发送。

验收： - 所有摇杆和开关映射正确； - 中位、最大、最小值正确； -
飞控回传/调试值与预期一致； - USB 断开检测有效。

## 阶段 4：飞控 MAVLink 输入适配

任务： - 核实或完善 Message 70 接收； - 适配到原有 CRSF 后级内部 RC
格式； - 核实/实现 Message 84 接收； - 解析 NED XYZ 速度； -
实现独立超时保护； - 飞控内部模式选择输入源。

验收： - 原有 CRSF 工作不受影响； - MAVLink RC 通道进入原有 RC
处理逻辑； - Message 84 解析准确； - 坐标转换方向正确； -
失效保护有效； - 不重写速度环和姿态环。

## 阶段 5：地面位置控制

任务： - UWB 坐标标定； - map/ENU/NED 转换； - BODY_FRD
速度转世界速度； - 位置 P/PD 控制； - 速度和加速度限幅； -
目标到达判定。

验收： - 静态位置误差可测量； - 低速期望速度方向正确； - 飞控接收的 NED
速度方向正确； - 低速点到点运动可控； - 接近目标能够减速； -
到达后水平速度能够降至阈值。

## 阶段 6：RViz 自主飞行集成

任务： - RViz 目标点； - Goal Manager； - 任务状态； - 轨迹显示； -
目标取消； - 飞控模式切换； - 失联保护。

验收： - 点击目标点后能够自主移动； - 能在目标附近稳定停止； -
可连续执行多个目标点； - 可由飞控原有模式切换人工接管； -
UWB/图传异常时安全策略有效。

------------------------------------------------------------------------

# 12. Codex 执行规则

1.  先检查现有工程，再修改。
2.  不得假设飞控内部变量和数据结构。
3.  不得重写原有 CRSF 解码器。
4.  不得重写原有速度环和姿态环。
5.  地面站固定发送
    HEARTBEAT、RC_CHANNELS_OVERRIDE、SET_POSITION_TARGET_LOCAL_NED。
6.  飞控固定发送 HEARTBEAT、ODOMETRY。
7.  不得改用 Message 420。
8.  不建立地面站 MANUAL/AUTO Control Mux。
9.  飞控自行完成模式选择。
10. UWB 只接入地面站，不发送 UWB XY 给飞控。
11. 第一阶段不使用 UWB Z 控制。
12. Pocket 通过 USB HID 接入 Ubuntu。
13. 地面站功能使用 ROS 2 C++。
14. 使用官方 nlink_parser2，除非有明确原因，不修改官方驱动源码。
15. NED 和 BODY_FRD 速度不得直接混用。
16. 不得把无效传感器数据当成有效值。
17. 所有控制数据必须有超时保护。
18. 不得在未验证安全措施前自动解锁或启动扑翼机构。
19. 每阶段结束输出代码变更摘要、测试结果、未解决问题和验收报告。
20. 若实际工程与本计划冲突，先报告差异、证据和建议，等待用户确认后再改变架构。

------------------------------------------------------------------------

# 13. 尚待检查和实测的参数

  参数                            当前状态
  ------------------------------- --------------------
  Ubuntu 版本                     待检查
  ROS 2 发行版                    待检查
  飞控 System ID / Component ID   待检查
  MAVLink 2 支持情况              待检查
  Message 70 现有实现             待检查
  Message 84 接收实现             待检查
  Message 331 发送实现            待检查
  CRSF 内部通道格式               待检查
  原有 RC 通道顺序和标定          待检查
  速度环输入坐标系和单位          待检查
  光流速度轴向和正负号            待实测
  光流质量有效范围                待检查
  高度参考原点                    待检查
  飞控垂直速度/高度保持能力       待检查
  Pocket USB HID 轴顺序           待实测
  Pocket 开关映射                 待实测
  LinkTrack Anchor 输出协议       待确认
  LinkTrack Tag ID                待确认
  UWB 串口名和波特率              待检查
  UWB 坐标系方向                  待标定
  UWB 实际更新率和噪声            待实测
  图传 UART 波特率                沿用现有配置并核验
  MAVLink 实际收发频率            待实测
  UDP 丢包和延迟                  待实测

------------------------------------------------------------------------

# 14. 最终验收流程

1.  Ubuntu 启动 ROS 2；
2.  nlink_parser2 启动并读取 Anchor；
3.  地面站获取飞行器 Tag 的 XY；
4.  Pocket 通过 USB HID 连接 Ubuntu；
5.  joy_node 读取摇杆和开关；
6.  地面站持续发送 HEARTBEAT；
7.  地面站持续发送 RC_CHANNELS_OVERRIDE；
8.  地面站持续发送 SET_POSITION_TARGET_LOCAL_NED；
9.  飞控内部根据模式选择控制输入；
10. 飞控持续回传 HEARTBEAT 和 ODOMETRY；
11. 地面站使用 UWB XY 和 ODOMETRY 机体速度反馈进行位置控制；
12. RViz 点击目标点；
13. 飞行器自主移动；
14. 到达目标点后减速并稳定停止；
15. 用户可通过飞控内部模式切换进行人工接管。

**最终目标：在受控、平整的室内实验环境中，实现微型扑翼飞行器基于 UWB
的自主点到点飞行。**

------------------------------------------------------------------------

# 附录：官方参考资料

-   MAVLink Common Messages：\
    https://mavlink.io/en/messages/common.html
-   RC_CHANNELS_OVERRIDE：\
    https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
-   SET_POSITION_TARGET_LOCAL_NED：\
    https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
-   ODOMETRY：\
    https://mavlink.io/en/messages/common.html#ODOMETRY
-   Nooploop nlink_parser2：\
    https://github.com/nooploop-dev/nlink_parser2
-   ROS 2 joy：\
    https://index.ros.org/p/joy/

**项目实施原则：先审查、再验证接口；先单项测试、再系统集成；先地面安全测试、再低速受控飞行。**

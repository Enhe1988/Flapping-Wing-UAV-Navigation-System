# 软件验证摘要（不是实飞数据）

这里只保留上传前已有的少量通过结果，完整合成实验输出留在被忽略的log/experiments_demo。日志中的本机路径和临时目录指向当时运行现场，不代表这些合成数据也上传了。

| 文件 | 验证内容 |
|---|---|
| tracking-log-build.log | 最近一次完整工作区构建通过 |
| tracking-log-regression.log | 103项C++/ROS测试，无失败/跳过 |
| tracking-log-gates.log | figure8到TRACKING开录、waypoint对齐开录、关闭新开关恢复旧行为；迟到旧点不混入 |
| tracking-log-paper-regression.log | 完整旧遥测日志、真实节点回环、Ctrl+C及导航子进程退出自动保存 |
| tracking-log-mission-regression.log | 合成直线与八字，从实际到起点才TRACKING/开录，自动出图 |
| attitude-pid-integration.log | 隔离UDP角度报文、目标角限幅、UWB差分制动、积分与断流复位；文件内部分旧标签沿用当时文字 |
| uwb-filter-comparison-tests.log | 滤波前后图保留原始点，共用坐标/参考，断流断线，缺测不伪造 |
| uwb-comparison-regression.log | 同次原始UWB/定位选择输出两图与数据完整性 |
| uwb-export-regression.log | 导出互斥、恢复、NaN和原始字段保存 |

这些结果来自私有DDS域和本机回环/合成输入，不访问实体飞控，不能当作控制性能证明。测试后用户又调整过本地PID/轨迹配置；这里证明实现的已测行为，不声称所有后续参数均完成实飞整定。

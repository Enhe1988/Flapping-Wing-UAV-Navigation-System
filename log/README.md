# 实验数据与调试证据归档

本目录将实机原始记录与分析、验证摘要一起保存在Git中。原始CSV直接可读，不用压缩包替代；历史会话和文件按原字节保留，归档脚本只新增索引及校验清单。

| 路径 | 数据性质与用途 |
|---|---|
| [experiments/](experiments/README.md) | 29份真实设备实验会话，可能包括地面检查、手动干预或未完成轨迹；含CSV、既有PNG/PDF和参数快照 |
| [velocity_tests/](velocity_tests/) | 独立速度阶跃/链路实测CSV，含约4秒现象排查的电脑输入/UDP监测记录；不是当前角度控制的实测速度反馈 |
| [analysis/](analysis/) | 指定飞行记录的离线分析，含阶段裁剪、UWB差分估计、滤波重放；保留分析脚本和说明 |
| flight_follow_analysis_20261124、flight_response_analysis_20261125* | 早期航点/轨迹跟踪与目标消失、末尾零指令等调试分析 |
| [uwb_filter_review_20261125/](uwb_filter_review_20261125/) | 已保存的离线滤波重放；CSV为派生点，不是在线观测原件 |
| [validation/](validation/README.md) | 合成软件验证摘要，明确不是实飞证据 |

## 核对原始数据

工程根目录执行：

```bash
python3 scripts/index_experiments.py --verify
# 或仅在出现校验问题时输出
sha256sum --quiet -c log/checksums.sha256
```

[checksums.sha256](checksums.sha256)覆盖本次上传的日志、图表、分析文件及元数据（不对校验文件自身求哈希）；[archive_inventory.json](archive_inventory.json)汇总数量、大小和manifest行数核对结果。后续实验增加或报告重新导出后，再运行`python3 scripts/index_experiments.py`更新归档索引。它不编辑任何实验原始CSV。

## 如何解释历史记录

- 以每份session.json的模式、开录触发、文件行数和parameters/快照解释数据，不用当前PID覆盖历史参数。
- 2026-11-26开始，line/figure8默认TRACKING开录；此前常从航向对齐开始，可能包含准备、接入和落地。开录后仍到Ctrl+C才结束。
- trajectory.csv为原始map UWB；uwb_filtered.csv为当时在线滤波点；control_position.csv为控制采用位置；reference.csv为真正接受/执行的参考。
- 未回传的速度、实际roll/pitch或角速度保持NaN/空表，不能把目标角当作实测姿态。sent表示交给电脑UDP，不是飞控接收确认。
- localization_comparison中的点副本按原样保留。关闭光流接管时，图表不能被解读为已证明光流补偿有效。
- 有的早期会话缺少后来新增的CSV或报告；不补造、不删除这类记录，索引如实列出问题。

构建目录、colcon流水日志、log/ros终端输出、可再生成的完整合成实验、缓存和外部Git依赖工作副本不上传。需要重现实验的软件检查可运行仓库内scripts/下的验证脚本；需要当前操作步骤见[主README](../README.md)。

# 本机 Milestone 6 运行记录

输入：`resources/task_4.mp4`

OpenCV：4.5.4；尺寸：1440×1080；FPS：30；完整处理帧数：1800。

两个输出视频均已重新解码检查尺寸、FPS 和帧数。该检查不代表检测或身份全部正确。

| 统计 | 数量 |
|---|---:|
| detected | 1322 |
| lost | 168 |
| waiting | 310 |
| R detected | 1572 |
| 至少两个有效候选的帧 | 763 |
| 新建 ID / ACQUIRED | 17 |
| 同 ID 恢复 / REACQUIRED | 31 |
| 释放 ID / RELEASED | 16 |

## 实际参数

HSV H=[0,30], S>=100, V>=70; open=0, close=3。

Circle radius=[25,60], minCenterDistance=45, Hough=18, innerRatio=0.55, gapRatio=0.79, outer>=0.65, inner>=0.6, gap<=0.7。

R area=[120,600], width=[12,36], height=[10,34], aspect=[0.85,1.6], extent=[0.58,0.9], H<=13, V>=110, surround<=0.35, score>=0.72, ambiguity=0.08, distanceRatio=[3.5,8]。

Tracker maxLostFrames=8, angleGateDeg=20, gateGrowthDeg=2, maxAngleGateDeg=30, maxDistanceChange=0.3, maxRadiusChange=0.35, ambiguityMargin=0.1, motionAlpha=0.4, maxStepDeg=7。

## 身份与失败规则

首次任选一个有效候选，本实现按质量选择；已锁定后只按关联规则更新。第 8 个连续失败帧显示 RELEASED，下一帧才允许新建 ID。短暂丢失恢复为 REACQUIRED，沿用旧 ID；释放后的 ACQUIRED 使用新 ID。ID 表示一次连续锁定片段，不是永久扇叶编号；每个视频从 1 开始。

R 失效同样计为无有效观测。没有被匹配的目标坐标/角度留空，不绘制旧结果。角度增量仅用于图像序列关联，不是现场角速度；video_time_s 是播放时间。

## 局限与人工验收

M4/M5 的漏检、误检仍会传递到跟踪器。强透视、突变运动、长时间丢失后相似目标占据相同方位均可能使本方法失败。没有人工逐帧真值，以上不是准确率、召回率或 ID 切换率。请对照 tracking_events.csv 检查每次 ACQUIRED/RELEASED，并在总报告中补充误检例子。

输出：[标注视频](recognition_overlay.mp4)、[二值化](binary_process.mp4)、[逐帧日志](tracking.csv)、[事件日志](tracking_events.csv)。

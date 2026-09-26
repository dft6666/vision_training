# Milestone 7 本机结构性核验

本文件由本机输入、输出视频和 M6 日志生成。每次重新运行会替换本自动文件。
**PASS 仅表示这里执行的结构性检查通过，不代表识别准确、物理身份正确或整份作业验收通过。**

| 输入 | 状态 | 输入/标注解码帧数 | 尺寸/FPS |
|---|---|---|---|
| task_3 | PASS | 796/796 | 1440×1080 / 30.0 |
| task_4 | PASS | 1800/1800 | 1440×1080 / 30.0 |

| 视频 | 文件 | 实际大小 / MiB |
|---|---|---:|
| task_3 | task_3.mp4 | 7.373 |
| task_3 | recognition_overlay.mp4 | 10.838 |
| task_3 | binary_process.mp4 | 13.268 |
| task_4 | task_4.mp4 | 14.946 |
| task_4 | recognition_overlay.mp4 | 24.835 |
| task_4 | binary_process.mp4 | 28.408 |

## task_3

输入和标注：全帧可解码、尺寸一致、FPS 一致、相对播放时间戳一致。
逐帧日志：帧号连续；有效观测使用当前 R；失败帧位置为空；ID 与丢失/释放规则一致。
事件日志与摘要统计和 tracking.csv 相符。
这些检查不能证明画面内容没有重复/错序，也不能判断绿色圆是否始终是同一物理目标。

## task_4

输入和标注：全帧可解码、尺寸一致、FPS 一致、相对播放时间戳一致。
逐帧日志：帧号连续；有效观测使用当前 R；失败帧位置为空；ID 与丢失/释放规则一致。
事件日志与摘要统计和 tracking.csv 相符。
这些检查不能证明画面内容没有重复/错序，也不能判断绿色圆是否始终是同一物理目标。

## 当前环境（不是历史运行环境证明）

- Python: `3.10.12`
- g++: `g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0`
- CMake: `cmake version 3.22.1`
- Git: `git version 2.34.1`
- ffprobe: `ffprobe version 4.4.2-0ubuntu0.22.04.1 Copyright (c) 2007-2021 the FFmpeg developers`

检测时的 OpenCV 版本和运行参数来自各视频的 tracking_summary.md。
manifest.json 的 SHA256 是检查时的快照，不是源码与旧视频自动匹配的证明。
修改源码或参数后需重新编译、重跑两个视频，再重新运行本工具。

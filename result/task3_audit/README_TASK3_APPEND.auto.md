
## 任务 3：真实能量机关识别与稳定跟踪

任务 3 的识别和跟踪程序使用 C++17、OpenCV、CMake；沿用任务 1、2 的工程和根目录构建配置。
独立核验脚本使用 Python 3 标准库与 ffprobe，不参与识别算法，不需要 pip/conda 环境。

### 构建与运行

在工程根目录执行：

```bash
cmake -S . -B build
cmake --build build --target task3_windmill -j4
./build/task3_windmill resources/task_3.mp4 result/task3_windmill/task_3
./build/task3_windmill resources/task_4.mp4 result/task3_windmill/task_4
python3 tools/finalize_task3.py --root .
```

输入采用统一发放素材，放到 resources/；同一个可执行程序通过两个路径参数处理不同视频。
参数位置：include/task3_windmill/detector.hpp、tracker.hpp；本次实际参数写在各自 tracking_summary.md。

| 视频 | 场景 | 尺寸 | FPS | 解码帧数 |
|---|---|---|---:|---:|
| task_3.mp4 | 小能量机关 | 1440×1080 | 30.0 | 796 |
| task_4.mp4 | 大能量机关 | 1440×1080 | 30.0 | 1800 |

### 输出与说明

[任务 3 跟踪说明](result/task3_tracking_result.md)；[结构性核验](result/task3_audit/audit.md)。

- task_3：[完整标注视频](result/task3_windmill/task_3/recognition_overlay.mp4)、[运行参数与统计](result/task3_windmill/task_3/tracking_summary.md)、[逐帧日志](result/task3_windmill/task_3/tracking.csv)。
  [二值化过程](result/task3_windmill/task_3/binary_process.mp4)。
- task_4：[完整标注视频](result/task3_windmill/task_4/recognition_overlay.mp4)、[运行参数与统计](result/task3_windmill/task_4/tracking_summary.md)、[逐帧日志](result/task3_windmill/task_4/tracking.csv)。
  [二值化过程](result/task3_windmill/task_4/binary_process.mp4)。

检测状态与 ID 是程序输出，不据此宣称检测准确率或物理身份零错误。等待/丢失帧完整保留，失败时不把旧坐标绘制成当前有效目标。
该章节是总 README 的追加内容，不替代任务 1 分析、任务 2 参数说明和全工程依赖。

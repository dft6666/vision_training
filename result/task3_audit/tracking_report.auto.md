# 任务 3：真实能量机关识别与稳定跟踪（本机报告草稿）

> 本稿根据当前本机文件自动整理。先复制到 `result/task3_tracking_result.md` 再编辑。
> 下列链接按最终报告位置计算。提交前填写所有【待人工填写】，核对方法与最终源码一致。

## 1. 输入与任务范围

使用发放的 task_3.mp4、task_4.mp4，分别对应小能量机关与大能量机关，文件放入 resources/。
使用同一个 C++17 / OpenCV 程序处理两段素材。真实视频不做现场角速度拟合，日志时间仅为播放时刻。

| 视频 | 场景 | 尺寸 | FPS | 输入解码帧数 | 标注解码帧数 | 结构检查 |
|---|---|---|---:|---:|---:|---|
| task_3.mp4 | 小能量机关 | 1440×1080 | 30.0 | 796 | 796 | PASS |
| task_4.mp4 | 大能量机关 | 1440×1080 | 30.0 | 1800 | 1800 | PASS |

仅在上表 PASS 时，可表述本次尺寸、帧数、FPS 和相对播放时序核对通过；FAIL 时需先修复。
不据此声称逐像素无损、音轨保留、定位准确或物理身份零错误。

## 2. 检测与关联方法（依据 M3–M6 方案，需核对最终源码）

颜色检测：BGR 转 HSV，范围分割后进行可选形态学处理，保留原始掩膜供结构检查。
候选目标：霍夫圆粗选，再检查外环、内环和两环之间的空隙；圆环质量分数不是概率。
R 检测：逐帧筛选小轮廓的面积、宽高、长宽比、颜色、邻域和与目标的距离关系，采用外接框中心估计 R 标中心。
相对几何：dx=target_x-r_x，dy=target_y-r_y；theta=atan2(-dy,dx)。共同二维平移被相减消去，但未校正透视。
稳定关联：首次按质量选一个有效候选；已有锁定后按相对角度、中心距离变化和目标小圆半径变化关联，不因另一个候选质量更高而抢选。短期图像角度增量仅作关联参考，不是物理运动预测结果。
ID 表示连续锁定片段；释放后的新锁定使用新 ID，不是物理扇叶永久编号。

## 3. 本机运行参数

### task_3

运行日志所记 OpenCV：`4.5.4`。

```text
HSV H=[0,30], S>=100, V>=70; open=0, close=3。

Circle radius=[25,60], minCenterDistance=45, Hough=18, innerRatio=0.55, gapRatio=0.79, outer>=0.65, inner>=0.6, gap<=0.7。

R area=[120,600], width=[12,36], height=[10,34], aspect=[0.85,1.6], extent=[0.58,0.9], H<=13, V>=110, surround<=0.35, score>=0.72, ambiguity=0.08, distanceRatio=[3.5,8]。

Tracker maxLostFrames=8, angleGateDeg=20, gateGrowthDeg=2, maxAngleGateDeg=30, maxDistanceChange=0.3, maxRadiusChange=0.35, ambiguityMargin=0.1, motionAlpha=0.4, maxStepDeg=7。
```

本次 maxLostFrames=8：第 8 个连续失败帧释放；释放帧不立即重选，下一帧才允许新锁定。
未达门限时保留旧 ID；恢复关联为 REACQUIRED，保持旧 ID。R 失效也算无有效观测。
失败帧不删除，目标位置与角度字段留空，不用旧位置或关联参考冒充当前观测。

### task_4

运行日志所记 OpenCV：`4.5.4`。

```text
HSV H=[0,30], S>=100, V>=70; open=0, close=3。

Circle radius=[25,60], minCenterDistance=45, Hough=18, innerRatio=0.55, gapRatio=0.79, outer>=0.65, inner>=0.6, gap<=0.7。

R area=[120,600], width=[12,36], height=[10,34], aspect=[0.85,1.6], extent=[0.58,0.9], H<=13, V>=110, surround<=0.35, score>=0.72, ambiguity=0.08, distanceRatio=[3.5,8]。

Tracker maxLostFrames=8, angleGateDeg=20, gateGrowthDeg=2, maxAngleGateDeg=30, maxDistanceChange=0.3, maxRadiusChange=0.35, ambiguityMargin=0.1, motionAlpha=0.4, maxStepDeg=7。
```

本次 maxLostFrames=8：第 8 个连续失败帧释放；释放帧不立即重选，下一帧才允许新锁定。
未达门限时保留旧 ID；恢复关联为 REACQUIRED，保持旧 ID。R 失效也算无有效观测。
失败帧不删除，目标位置与角度字段留空，不用旧位置或关联参考冒充当前观测。

## 4. 观测状态统计（不是准确率）

| 视频 | detected | lost | waiting | 新建 ID | 同 ID 恢复 | 释放 ID |
|---|---:|---:|---:|---:|---:|---:|
| task_3 | 548 | 47 | 201 | 5 | 7 | 5 |
| task_4 | 1322 | 168 | 310 | 17 | 31 | 16 |

上述数量来自程序输出状态，不是人工真值；未标注真实目标，故不报告准确率、召回率、ID 切换率或“满分”。

## 5. 人工复查与典型事件

复查导航见 [自动生成的回看片段](task3_audit/manual_review_windows.md)。它只指出程序事件，不证明事件合理。

| 检查项 | 视频与帧范围 | 对照原图观察到的事实 | 判定与原因 |
|---|---|---|---|
| 两目标同时有效时是否保持原目标 | 【待人工填写】 | 【待人工填写】 | 【待人工填写】 |
| 短暂丢失后是否恢复同一实体和 ID | 【待人工填写】 | 【待人工填写】 | 【待人工填写】 |
| 释放和重新选择是否合理 | 【待人工填写】 | 【待人工填写】 | 【待人工填写】 |
| 至少一个已知漏检/误检/误关联例子 | 【待人工填写】 | 【待人工填写】 | 【待人工填写】 |

当本次录像中未出现某类情形，应写清“本次未观察到”，不能编造成功例子。
完整观看结论：【待人工填写：两个视频是否全程播放并对照原图，检查日期、发现的问题】。

## 6. 局限与改进

当前方法依赖颜色、圆环形状和经验门限；R/圆检测错误会传入跟踪器。
强透视、灯光闪烁、相机突变、相似目标在相近方位、长时间丢失均有误关联风险。
ID 文本不变并不证明物理身份始终正确。
本机调整及效果：【待人工填写：参数、修改原因、复跑两个视频后的收益与代价；未调参如实填写】。

## 7. 构建、运行与结果索引

从项目根目录执行：

```bash
cmake -S . -B build
cmake --build build --target task3_windmill -j4
./build/task3_windmill resources/task_3.mp4 result/task3_windmill/task_3
./build/task3_windmill resources/task_4.mp4 result/task3_windmill/task_4
```

- task_3：[标注视频](task3_windmill/task_3/recognition_overlay.mp4)、[本机运行记录](task3_windmill/task_3/tracking_summary.md)、[逐帧日志](task3_windmill/task_3/tracking.csv)、[事件日志](task3_windmill/task_3/tracking_events.csv)。
- task_4：[标注视频](task3_windmill/task_4/recognition_overlay.mp4)、[本机运行记录](task3_windmill/task_4/tracking_summary.md)、[逐帧日志](task3_windmill/task_4/tracking.csv)、[事件日志](task3_windmill/task_4/tracking_events.csv)。

[自动结构检查](task3_audit/audit.md)。输入来源为随作业发放的 resources/task_3.mp4 和 resources/task_4.mp4。
素材或视频若改为附件/外部大文件交付，应在此与总 README 写明实际获取方式并验证可访问。

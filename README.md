# Vision Training

## 项目简介

本项目为第二次培训作业，目前完成 **Task 1：OpenCV 图片处理**。

Task 1 使用 C++ 和 OpenCV 对郁金香图片进行基础图像处理，主要包括：

- 图像读取与灰度转换
- HSV 颜色空间转换与 H、S、V 通道分离
- 均值滤波、高斯滤波和中值滤波
- 基于 HSV 的红色区域提取
- 腐蚀、膨胀、开运算和闭运算
- 轮廓提取与面积筛选
- 外接矩形绘制及轮廓面积标注
- 圆、矩形和文字绘制
- 图像旋转
- 图像裁剪
---

# 1. 环境与依赖

本任务开发环境为：

- Ubuntu Linux
- C++17
- OpenCV 4.5.4
- CMake
- GNU C++ Compiler

主要依赖的软件包包括：

```text
build-essential
cmake
libopencv-dev
pkg-config
```


---

# 2. 工程结构

当前 Task 1 相关工程结构如下：

```text
vision_training/
├── CMakeLists.txt
├── README.md
├── resources/
│   └── test_image.jpg
├── src/
│   └── task1_image/
│       └── main.cpp
├── result/
│   └── task1_images/
└── build/
```

其中：

- `resources/`：存放输入素材；
- `src/task1_image/`：存放 Task 1 源代码；
- `result/task1_images/`：存放 Task 1 生成的图片；
- `build/`：CMake 编译产生的构建文件，不属于源代码。

---

# 3. 输入图片

Task 1 使用的输入图片为：

```text
resources/test_image.jpg
```

程序中通过：

```cpp
Mat img = imread("resources/test_image.jpg");
```

读取图片。

若读取失败，则通过：

```cpp
if (img.empty())
```

检查并终止程序。

当前图片尺寸为：

```text
Width  = 1280
Height = 853
```

---

---

# 4. Task 1：OpenCV 图片处理

## 4.1 灰度转换

原始图片由 OpenCV 以 BGR 彩色格式读取。

使用：

```cpp
cvtColor(img, gray, COLOR_BGR2GRAY);
```

将三通道 BGR 图像转换为单通道灰度图。

输出：

```text
result/task1_images/gray.png
```

灰度图只保留图像的亮度信息，不再直接包含颜色信息，可以减少数据维度，也常用于后续的阈值处理、梯度计算和边缘检测。

---

## 4.2 HSV 颜色空间转换

使用：

```cpp
cvtColor(img, hsv, COLOR_BGR2HSV);
```

将原始 BGR 图像转换为 HSV 颜色空间。

HSV 三个通道分别表示：

- `H`：Hue，色相，主要表示颜色类别；
- `S`：Saturation，饱和度，表示颜色的鲜艳程度；
- `V`：Value，明度，表示像素的亮暗程度。

随后使用：

```cpp
split(hsv, hsvChannels);
```

将 HSV 图像分成三个单通道图像。

---

# 5. 滤波处理

为了比较不同图像平滑方法，本任务分别实现均值滤波、高斯滤波和中值滤波。

## 5.1 均值滤波

代码：

```cpp
blur(img, meanImg, Size(5, 5));
```

参数：

```text
Kernel Size = 5 × 5
```

均值滤波使用邻域中所有像素的平均值替换当前像素。

其主要作用是平滑图像中的局部变化和噪声，但由于邻域中所有像素具有相同权重，图像中的真实边缘和细节也会被一定程度模糊。

---

## 5.2 高斯滤波

代码：

```cpp
GaussianBlur(
    img,
    gaussianImg,
    Size(5, 5),
    1.5
);
```

参数：

```text
Kernel Size = 5 × 5
sigmaX      = 1.5
```
高斯滤波同样进行邻域平均，但不同位置具有不同权重。

距离中心像素越近的像素权重越高，因此相比简单的均值滤波，高斯滤波的平滑过渡更加自然。

在本图中，高斯滤波能够减少局部细小变化，同时主要花瓣轮廓仍然可以较清楚地观察到。

---

## 5.3 中值滤波

代码：

```cpp
medianBlur(img, medianImg, 5);
```

参数：

```text
Kernel Size = 5
```
中值滤波不是计算邻域平均值，而是将邻域像素排序后取中位数作为新的像素值。

该方法对孤立异常像素，尤其是椒盐噪声具有较好的抑制效果，同时通常能够比简单均值滤波更好地保留明显边缘。

---

## 5.4 三种滤波结果对比

三种滤波均能使图像更加平滑，但处理方式不同：

| 方法 | 当前参数 | 主要特点 |
| --- | --- | --- |
| 均值滤波 | 5×5 | 平滑明显，但容易模糊边缘 |
| 高斯滤波 | 5×5，sigmaX=1.5 | 加权平滑，过渡较自然 |
| 中值滤波 | 5 | 对孤立噪声抑制较好，并能够较好保持边缘 |

实际观察郁金香花瓣边缘可以发现，滤波在抑制细小变化的同时，也会不同程度削弱花瓣纹理和边缘细节，因此滤波核不宜设置过大。

---

# 6. HSV 红色区域提取

## 6.1 基本原理

为了提取图像中的红色区域，首先将 BGR 图像转换到 HSV 空间。

由于 OpenCV 8-bit HSV 中：

```text
H ∈ [0, 179]
S ∈ [0, 255]
V ∈ [0, 255]
```

而红色位于 H 色相环的首尾两端，因此使用两个 H 区间分别提取，再将两部分合并。

---

## 6.2 当前 HSV 阈值

### 第一段红色区域

```cpp
inRange(
    hsv,
    Scalar(0, 100, 100),
    Scalar(18, 255, 255),
    maskLow
);
```

对应：

```text
H:   0 ~ 18
S: 100 ~ 255
V: 100 ~ 255
```
注：一开始参数为0~10，但是与实际花样区别较大，远处的模糊的花并未考虑进红色区域内，因此调高范围让结果更可信
### 第二段红色区域

```cpp
inRange(
    hsv,
    Scalar(170, 100, 100),
    Scalar(179, 255, 255),
    maskHigh
);
```

对应：

```text
H: 170 ~ 179
S: 100 ~ 255
V: 100 ~ 255
```

随后通过：

```cpp
bitwise_or(maskLow, maskHigh, redMask);
```

将两个 mask 合并。

输出：

```text
result/task1_images/red_mask.png
```

---

## 6.3 Mask 的含义

`inRange()` 对每一个像素判断其 H、S、V 是否全部位于给定范围。

若满足条件：

```text
mask pixel = 255
```

显示为白色。

若不满足：

```text
mask pixel = 0
```

显示为黑色。

因此：

```text
白色区域 = 被认为属于目标红色范围
黑色区域 = 不属于当前红色范围
```

---

# 7. 形态学处理

HSV 分割后的 mask 可能存在：

- 小白色噪声；
- 前景内部的小黑洞；
- 较细的断裂；
- 不连续的目标区域。

因此对 `redMask` 进一步进行形态学处理。

## 7.1 结构元素

代码：

```cpp
Mat kernel = getStructuringElement(
    MORPH_RECT,
    Size(5, 5)
);
```

使用：

```text
Shape       = MORPH_RECT
Kernel Size = 5 × 5
```

即使用一个 5×5 的矩形结构元素。

---

## 7.2 腐蚀

代码：

```cpp
erode(redMask, eroded, kernel);
```

在当前二值 mask 中：

```text
白色 = 前景
黑色 = 背景
```

腐蚀会使白色前景区域缩小。

其效果包括：

- 使目标边缘向内收缩；
- 去掉部分较小的白色噪声；
- 但也可能使较细的目标区域消失。

---

## 7.3 膨胀

代码：

```cpp
dilate(redMask, dilated, kernel);
```

输出：

```text
result/task1_images/dilate.png
```

膨胀会扩大白色前景区域。

其效果包括：

- 扩大目标；
- 填补部分很小的间隙；
- 连接距离较近的白色区域；
- 但也可能使原本相邻的不同区域连接在一起。

---

## 7.4 开运算

代码：

```cpp
morphologyEx(
    redMask,
    opened,
    MORPH_OPEN,
    kernel
);
```

输出：

```text
result/task1_images/open.png
```

开运算的过程为：

```text
腐蚀 → 膨胀
```

由于较小的白色噪声可能在腐蚀阶段直接消失，后续膨胀无法将其恢复，因此开运算主要用于去除较小的白色前景噪声。

---

## 7.5 闭运算

代码：

```cpp
morphologyEx(
    redMask,
    closed,
    MORPH_CLOSE,
    kernel
);
```

输出：

```text
result/task1_images/close.png
```

闭运算的过程为：

```text
膨胀 → 腐蚀
```

膨胀首先填补部分小黑洞以及较窄的断裂，随后腐蚀将整体尺寸向原始目标恢复。

因此闭运算适合：

- 填补目标内部的小黑洞；
- 连接较窄的断裂；
- 提高连通区域的完整性。

本任务后续轮廓检测使用闭运算后的结果。

---

# 8. 轮廓提取与面积筛选

## 8.1 轮廓输入

代码中使用：

```cpp
Mat contourInput = closed.clone();
```

因此轮廓检测基于闭运算后的 mask。

随后调用：

```cpp
findContours(
    contourInput,
    contours,
    RETR_EXTERNAL,
    CHAIN_APPROX_SIMPLE
);
```

参数含义：

### RETR_EXTERNAL

只保留最外层轮廓。

### CHAIN_APPROX_SIMPLE

压缩位于直线段上的冗余轮廓点，减少轮廓数据量。

---

## 8.2 轮廓面积筛选

使用：

```cpp
double area = contourArea(contours[i]);
```

计算每个轮廓的面积。

当前面积阈值设置为：

```cpp
const double minArea = 500.0;
```

当：

```cpp
area < 500.0
```

时：

```cpp
continue;
```

直接忽略该轮廓。

因此：

```text
Minimum Contour Area = 500 pixels
```

设置面积阈值的目的是过滤面积很小、较可能属于噪声的红色连通区域。

---

## 8.3 绘制轮廓

筛选后的轮廓使用：

```cpp
drawContours(
    contourResult,
    contours,
    static_cast<int>(i),
    Scalar(0, 255, 0),
    2
);
```

进行绘制。

颜色：

```text
B = 0
G = 255
R = 0
```

因此轮廓使用绿色显示。

---

## 8.4 外接矩形

使用：

```cpp
Rect box = boundingRect(contours[i]);
```

计算每个轮廓的轴对齐外接矩形。

随后：

```cpp
rectangle(
    contourResult,
    box,
    Scalar(0, 0, 255),
    2
);
```

绘制矩形。

其颜色为：

```text
B = 0
G = 0
R = 255
```

因此使用红色矩形框。

这里的矩形对应的是：

```text
红色连通区域
```

而不是要求一个矩形严格对应一朵完整的郁金香。

---

## 8.5 轮廓面积标注

使用：

```cpp
string text =
    "Area=" + to_string(static_cast<int>(area));
```

构造面积文字。

随后通过：

```cpp
putText(...)
```

将每个筛选后的轮廓面积直接写在结果图上。

因此轮廓面积信息可以直接在：

```text
result/task1_images/contours_boxes.png
```

中查看。

输出：

```text
result/task1_images/contours_boxes.png
```

---

# 9. OpenCV 基础绘图

为了完成圆、矩形和文字绘制，首先创建原图副本：

```cpp
Mat drawing = img.clone();
```

所有绘制操作作用于 `drawing`，从而避免直接修改原始图像 `img`。

---

## 9.1 绘制圆

圆心设置为图像中心：

```cpp
Point center(
    img.cols / 2,
    img.rows / 2
);
```

绘制：

```cpp
circle(
    drawing,
    center,
    80,
    Scalar(255, 0, 0),
    3
);
```

参数：

```text
圆心：图像中心
半径：80 pixels
颜色：蓝色
线宽：3 pixels
```

OpenCV 默认使用 BGR 颜色顺序，因此：

```cpp
Scalar(255, 0, 0)
```

表示蓝色。

---

# 9.2 绘制矩形

矩形绘制代码：

```cpp
rectangle(
    drawing,
    Point(50, 50),
    Point(250, 180),
    Scalar(0, 255, 0),
    3
);
```

参数：

```text
左上角：(50, 50)
右下角：(250, 180)
颜色：绿色
线宽：3 pixels
```

其中：

```cpp
Scalar(0, 255, 0)
```

表示绿色。

---

## 9.3 绘制文字

代码：

```cpp
putText(
    drawing,
    "OpenCV Task 1",
    Point(50, 230),
    FONT_HERSHEY_SIMPLEX,
    1.0,
    Scalar(0, 0, 255),
    2
);
```

参数：

```text
Text      = OpenCV Task 1
Position  = (50, 230)
Font      = FONT_HERSHEY_SIMPLEX
FontScale = 1.0
Color     = Red
Thickness = 2
```

其中：

```cpp
Scalar(0, 0, 255)
```

表示红色。

最终输出：

```text
result/task1_images/drawing.png
```

该图片包含：

- 蓝色圆；
- 绿色矩形；
- 红色文字。

---

# 10. 图像旋转

要求将原图绕图像中心旋转 35°。

首先计算旋转中心：

```cpp
Point2f rotationCenter(
    img.cols / 2.0f,
    img.rows / 2.0f
);
```

随后使用：

```cpp
Mat rotationMatrix =
    getRotationMatrix2D(
        rotationCenter,
        35.0,
        1.0
    );
```

其中：

```text
Rotation Center = image center
Angle           = 35 degrees
Scale           = 1.0
```

然后：

```cpp
warpAffine(
    img,
    rotated,
    rotationMatrix,
    img.size()
);
```

完成仿射变换。
---

# 11. 图像裁剪

任务要求裁剪原图左上角的 1/4。

代码：

```cpp
Rect cropRegion(
    0,
    0,
    img.cols / 2,
    img.rows / 2
);
```

裁剪区域从原图左上角：

```text
(0, 0)
```

开始。

裁剪后的：

```text
width  = original width / 2
height = original height / 2
```

因此其面积为原图面积：

```text
1/2 × 1/2 = 1/4
```

随后：

```cpp
Mat cropped = img(cropRegion).clone();
```

得到裁剪图。
---

# 12. Task 1 输出文件

Task 1 共生成 16 张图片：

```text
result/task1_images/
├── gray.png
├── hsv_h.png
├── hsv_s.png
├── hsv_v.png
├── mean_filter.png
├── gaussian_filter.png
├── median_filter.png
├── red_mask.png
├── erode.png
├── dilate.png
├── open.png
├── close.png
├── contours_boxes.png
├── drawing.png
├── rotated_35deg.png
└── crop_top_left.png
```

各文件含义如下：

| 文件名 | 内容 |
| --- | --- |
| `gray.png` | 灰度图 |
| `hsv_h.png` | HSV 的 H 通道 |
| `hsv_s.png` | HSV 的 S 通道 |
| `hsv_v.png` | HSV 的 V 通道 |
| `mean_filter.png` | 均值滤波结果 |
| `gaussian_filter.png` | 高斯滤波结果 |
| `median_filter.png` | 中值滤波结果 |
| `red_mask.png` | HSV 红色分割得到的二值 mask |
| `erode.png` | 腐蚀结果 |
| `dilate.png` | 膨胀结果 |
| `open.png` | 开运算结果 |
| `close.png` | 闭运算结果 |
| `contours_boxes.png` | 轮廓、外接矩形及面积标注 |
| `drawing.png` | 圆、矩形及文字绘制 |
| `rotated_35deg.png` | 绕中心旋转 35° |
| `crop_top_left.png` | 原图左上角 1/4 |

---

# 13. Task 1 关键参数汇总
| 模块 | 参数 |
| --- | --- |
| 均值滤波 | Kernel = 5×5 |
| 高斯滤波 | Kernel = 5×5 |
| 高斯滤波 | sigmaX = 1.5 |
| 中值滤波 | Kernel = 5 |
| Red HSV 1 | H = 0~18 |
| Red HSV 1 | S = 100~255 |
| Red HSV 1 | V = 100~255 |
| Red HSV 2 | H = 170~179 |
| Red HSV 2 | S = 100~255 |
| Red HSV 2 | V = 100~255 |
| 形态学 Kernel | MORPH_RECT，5×5 |
| 最小轮廓面积 | 500 pixels |
| 绘制圆半径 | 80 pixels |
| 绘制矩形 | (50,50) → (250,180) |
| 旋转角度 | 35° |
| 旋转缩放 | 1.0 |
| 裁剪 | 左上角，宽高分别取原图 1/2 |

---

# 14. 总结

本任务完成了 OpenCV 图像处理流程中较基础且常用的操作，包括：

```text
原始图像
    ↓
颜色空间转换
    ↓
图像滤波
    ↓
HSV 颜色分割
    ↓
二值 Mask
    ↓
形态学处理
    ↓
轮廓提取
    ↓
面积筛选
    ↓
外接矩形及可视化
```

实验中可以观察到，传统图像处理方法中的参数与实际图像具有较强关联。

例如 HSV 阈值需要根据目标颜色、环境光照以及图像模糊程度调整；形态学核尺寸会影响连通区域的完整程度；面积阈值则决定哪些轮廓会被认为是有效目标。

通过本任务，对 OpenCV 中 `Mat`、颜色空间、滤波、mask、形态学操作、轮廓检测以及基本图像变换建立了初步理解。

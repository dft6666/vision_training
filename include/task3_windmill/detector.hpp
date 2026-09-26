#pragma once

#include "task3_windmill/types.hpp"

namespace task3
{

// 与 M3 相同。若你已经调过颜色阈值，保留自己验证过的数值。
struct MaskParameters
{
    int hMin = 0;
    int hMax = 30;
    int sMin = 100;
    int vMin = 70;
    int openKernel = 0;
    int closeKernel = 3;
};

struct CircleParameters
{
    // 这三个像素参数以 1080 像素图像高度为参考，代码按高度缩放。
    int minRadius = 25;
    int maxRadius = 60;
    double minCenterDistance = 45.0;

    double houghThreshold = 18.0;
    float innerRadiusRatio = 0.55f;
    float gapRadiusRatio = 0.79f;
    float minOuterSupport = 0.65f;
    float minInnerSupport = 0.60f;
    float maxGapSupport = 0.70f;
};

// R 检测的初始经验阈值：针对所给素材，非讲义指定常数。
struct RCenterParameters
{
    // 面积和长度以 1080 像素图像高度为参考，内部按尺度缩放。
    double minArea = 120.0;
    double maxArea = 600.0;
    double minWidth = 12.0;
    double maxWidth = 36.0;
    double minHeight = 10.0;
    double maxHeight = 34.0;
    double minAspect = 0.85;
    double maxAspect = 1.60;
    double minExtent = 0.58;
    double maxExtent = 0.90;
    double maxMeanHue = 13.0;
    double minMeanValue = 110.0;
    double maxSurroundingRatio = 0.35;
    double minShapeScore = 0.72;
    double ambiguityMargin = 0.08;
    // R->目标距离 / 目标圆半径；不是物理长度，也不是跟踪门限。
    double minDistanceRatio = 3.5;
    double maxDistanceRatio = 8.0;
};

cv::Mat createOrangeMask(
    const cv::Mat& frame,
    const MaskParameters& parameters
);

cv::Mat cleanOrangeMask(
    const cv::Mat& rawMask,
    const MaskParameters& parameters
);

std::vector<TargetCandidate> detectTargetCandidates(
    const cv::Mat& rawMask,
    const cv::Mat& cleanMask,
    const CircleParameters& parameters
);

RCenterResult detectRCenter(
    const cv::Mat& frame,
    const cv::Mat& rawMask,
    const std::vector<TargetCandidate>& targets,
    const RCenterParameters& parameters
);

bool isTargetGeometryPlausible(
    const TargetCandidate& target,
    const cv::Point2f& rCenter,
    const RCenterParameters& parameters
);

RelativeGeometry calculateRelativeGeometry(
    const TargetCandidate& target,
    const cv::Point2f& rCenter
);

} // namespace task3

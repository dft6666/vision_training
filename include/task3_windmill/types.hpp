#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <limits>

namespace task3
{

struct TargetCandidate
{
    cv::Point2f center{};
    float radius = 0.0f;
    // 几何匹配分数，不是“识别正确的概率”。
    float confidence = 0.0f;
    float outerSupport = 0.0f;
    float innerSupport = 0.0f;
    float gapSupport = 0.0f;
};

// R 标候选。score 是手工规则分数，不是分类概率。
struct RCenterCandidate
{
    cv::Point2f center{};
    cv::Rect box{};
    double contourArea = 0.0;
    double extent = 0.0;
    double meanHue = 0.0;
    double meanValue = 0.0;
    double surroundingRatio = 0.0;
    double score = 0.0;
    int compatibleTargets = 0;
};

// 本帧 R 检测结果。不保存上一帧的位置，也不使用固定中心。
struct RCenterResult
{
    bool detected = false;
    cv::Point2f center{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()
    };
    cv::Rect box{};
    double score = 0.0;
    std::string reason = "NO_R_CANDIDATE";
    std::vector<RCenterCandidate> candidates;
};

struct RelativeGeometry
{
    // 图像坐标：x 向右，y 向下。
    cv::Point2f imageOffset{};
    double distancePx = 0.0;
    // 相对于右方的图像平面角度，逆时针为正，单位 rad。
    double angleRad = 0.0;
};

// 跨帧跟踪的数据结构和类见 tracker.hpp；它们不依赖 OpenCV。

} // namespace task3

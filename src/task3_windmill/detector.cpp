#include "task3_windmill/detector.hpp"

#include <opencv2/imgproc.hpp>
#include <initializer_list>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace task3
{

// 1. 将 BGR 转成 HSV，根据颜色范围生成原始二值掩膜。
cv::Mat createOrangeMask(
    const cv::Mat& frame,
    const MaskParameters& parameters)
{
    if (frame.empty() || frame.type() != CV_8UC3)
    {
        throw std::runtime_error("Expected a nonempty 8-bit BGR frame.");
    }

    if (parameters.hMin < 0 || parameters.hMax > 179 ||
        parameters.hMin > parameters.hMax ||
        parameters.sMin < 0 || parameters.sMin > 255 ||
        parameters.vMin < 0 || parameters.vMin > 255)
    {
        throw std::runtime_error("Invalid HSV threshold parameters.");
    }

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat rawMask;
    cv::inRange(
        hsv,
        cv::Scalar(parameters.hMin, parameters.sMin, parameters.vMin),
        cv::Scalar(parameters.hMax, 255, 255),
        rawMask
    );

    return rawMask;
}

// 2. 可选开运算、闭运算；保留 rawMask，不直接修改它。
cv::Mat cleanOrangeMask(
    const cv::Mat& rawMask,
    const MaskParameters& parameters)
{
    if (rawMask.empty() || rawMask.type() != CV_8UC1)
    {
        throw std::runtime_error("Expected a nonempty single-channel mask.");
    }

    for (int size : {parameters.openKernel, parameters.closeKernel})
    {
        if (size != 0 && (size < 3 || size % 2 == 0))
        {
            throw std::runtime_error(
                "Kernel size must be 0 or an odd integer >= 3.");
        }
    }

    cv::Mat cleanMask = rawMask.clone();

    if (parameters.openKernel > 0)
    {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(parameters.openKernel, parameters.openKernel)
        );
        cv::morphologyEx(cleanMask, cleanMask, cv::MORPH_OPEN, kernel);
    }

    if (parameters.closeKernel > 0)
    {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(parameters.closeKernel, parameters.closeKernel)
        );
        cv::morphologyEx(cleanMask, cleanMask, cv::MORPH_CLOSE, kernel);
    }

    return cleanMask;
}


// 3. 沿圆周取 120 个方向，每个方向在半径附近检查 7 个点。
// 越多方向遇到白色像素，越像一个真正的发光圆环。
float calculateRingSupport(
    const cv::Mat& mask,
    const cv::Point2f& center,
    float radius,
    double tolerance)
{
    constexpr int angleSamples = 120;
    constexpr int radialSamples = 7;
    int hitDirections = 0;

    for (int i = 0; i < angleSamples; ++i)
    {
        const double angle = 2.0 * CV_PI * i / angleSamples;
        bool hit = false;

        for (int j = 0; j < radialSamples; ++j)
        {
            const double offset = -tolerance +
                2.0 * tolerance * j / (radialSamples - 1);
            const double sampleRadius = radius + offset;
            if (sampleRadius <= 0.0) continue;

            const int x = cvRound(
                center.x + sampleRadius * std::cos(angle));
            const int y = cvRound(
                center.y + sampleRadius * std::sin(angle));

            if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows &&
                mask.at<unsigned char>(y, x) != 0)
            {
                hit = true;
                break;
            }
        }

        if (hit) ++hitDirections;
    }

    return static_cast<float>(hitDirections) / angleSamples;
}

// 4. HoughCircles 只生成“像圆的区域”，不判断它是不是有效目标。
std::vector<cv::Vec3f> findCircleProposals(
    const cv::Mat& cleanMask,
    const CircleParameters& parameters)
{
    const double scale = cleanMask.rows / 1080.0;
    const int minRadius = std::max(2, cvRound(parameters.minRadius * scale));
    const int maxRadius = std::max(
        minRadius + 1, cvRound(parameters.maxRadius * scale));

    cv::Mat smoothMask;
    cv::GaussianBlur(cleanMask, smoothMask, cv::Size(5, 5), 1.0);

    std::vector<cv::Vec3f> proposals;
    cv::HoughCircles(
        smoothMask,
        proposals,
        cv::HOUGH_GRADIENT,
        1.0,
        std::max(1.0, parameters.minCenterDistance * scale),
        100.0,
        parameters.houghThreshold,
        minRadius,
        maxRadius
    );

    return proposals;
}

// 5. 检查外环、内环和两环之间的空隙；三个条件同时通过才保留。
std::vector<TargetCandidate> detectTargetCandidates(
    const cv::Mat& rawMask,
    const cv::Mat& cleanMask,
    const CircleParameters& parameters)
{
    if (rawMask.empty() || cleanMask.empty() ||
        rawMask.type() != CV_8UC1 || cleanMask.type() != CV_8UC1 ||
        rawMask.size() != cleanMask.size())
    {
        throw std::runtime_error("Invalid masks for circle detection.");
    }

    if (parameters.minRadius <= 0 ||
        parameters.maxRadius <= parameters.minRadius ||
        !std::isfinite(parameters.minCenterDistance) ||
        parameters.minCenterDistance <= 0.0 ||
        !std::isfinite(parameters.houghThreshold) ||
        parameters.houghThreshold <= 0.0 ||
        !std::isfinite(parameters.innerRadiusRatio) ||
        parameters.innerRadiusRatio <= 0.0f || parameters.innerRadiusRatio >= 1.0f ||
        !std::isfinite(parameters.gapRadiusRatio) ||
        parameters.gapRadiusRatio <= parameters.innerRadiusRatio ||
        parameters.gapRadiusRatio >= 1.0f ||
        !std::isfinite(parameters.maxGapSupport) ||
        parameters.maxGapSupport < 0.0f || parameters.maxGapSupport > 1.0f ||
        !std::isfinite(parameters.minOuterSupport) ||
        parameters.minOuterSupport < 0.0f || parameters.minOuterSupport > 1.0f ||
        !std::isfinite(parameters.minInnerSupport) ||
        parameters.minInnerSupport < 0.0f || parameters.minInnerSupport > 1.0f)
    {
        throw std::runtime_error("Invalid circle parameters.");
    }

    const auto proposals = findCircleProposals(cleanMask, parameters);
    const double tolerance = std::max(1.0, 3.0 * rawMask.rows / 1080.0);
    const double gapTolerance = std::max(0.5, rawMask.rows / 1080.0);
    std::vector<TargetCandidate> targets;

    for (const cv::Vec3f& circle : proposals)
    {
        TargetCandidate target;
        target.center = cv::Point2f(circle[0], circle[1]);
        target.radius = circle[2];

        target.outerSupport = calculateRingSupport(
            rawMask, target.center, target.radius, tolerance);
        target.innerSupport = calculateRingSupport(
            rawMask, target.center,
            target.radius * parameters.innerRadiusRatio, tolerance);

        target.gapSupport = calculateRingSupport(
            rawMask, target.center,
            target.radius * parameters.gapRadiusRatio, gapTolerance);

        if (target.outerSupport < parameters.minOuterSupport ||
            target.innerSupport < parameters.minInnerSupport ||
            target.gapSupport > parameters.maxGapSupport)
        {
            continue;
        }

        target.confidence =
            0.5f * (target.outerSupport + target.innerSupport);
        targets.push_back(target);
    }

    // 不按画面位置固定选择，不强行截取前 1 或前 2 个，不分配跟踪 ID。
    return targets;
}



// 6. 检查新增参数。输入错误时报告，不悄悄退回固定中心。
void validateRCenterParameters(const RCenterParameters& p)
{
    for (double value : {p.minArea, p.maxArea, p.minWidth, p.maxWidth,
         p.minHeight, p.maxHeight, p.minAspect, p.maxAspect,
         p.minExtent, p.maxExtent, p.maxMeanHue, p.minMeanValue,
         p.maxSurroundingRatio, p.minShapeScore, p.ambiguityMargin,
         p.minDistanceRatio, p.maxDistanceRatio})
    {
        if (!std::isfinite(value))
            throw std::runtime_error("Nonfinite R detection parameter.");
    }
    if (p.minArea <= 0 || p.maxArea <= p.minArea ||
        p.minWidth <= 0 || p.maxWidth < p.minWidth ||
        p.minHeight <= 0 || p.maxHeight < p.minHeight ||
        p.minAspect <= 0 || p.maxAspect < p.minAspect ||
        p.minExtent < 0 || p.maxExtent > 1 || p.maxExtent < p.minExtent ||
        p.maxMeanHue < 0 || p.maxMeanHue > 179 ||
        p.minMeanValue < 0 || p.minMeanValue > 255 ||
        p.maxSurroundingRatio < 0 || p.minShapeScore < 0 ||
        p.minShapeScore > 1 || p.ambiguityMargin < 0 ||
        p.minDistanceRatio <= 0 || p.maxDistanceRatio <= p.minDistanceRatio)
        throw std::runtime_error("Invalid R detection parameters.");
}

// 7. 全图提取 R 候选：大小、长宽比、紧实程度、颜色和邻域干扰。
// 这是启发式小标志检测，不是 OCR，也不能证明一个轮廓语义上一定是 R。
std::vector<RCenterCandidate> findRCenterCandidates(
    const cv::Mat& frame,
    const cv::Mat& rawMask,
    const RCenterParameters& p)
{
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat contourInput = rawMask.clone();
    cv::findContours(contourInput, contours,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double scale = rawMask.rows / 1080.0;
    std::vector<RCenterCandidate> candidates;
    for (const auto& contour : contours)
    {
        const double area = cv::contourArea(contour);
        const cv::Rect box = cv::boundingRect(contour);
        if (area < p.minArea * scale * scale ||
            area > p.maxArea * scale * scale ||
            box.width < p.minWidth * scale || box.width > p.maxWidth * scale ||
            box.height < p.minHeight * scale || box.height > p.maxHeight * scale)
            continue;

        const double aspect = double(box.width) / box.height;
        const double extent = area / (double(box.width) * box.height);
        if (aspect < p.minAspect || aspect > p.maxAspect ||
            extent < p.minExtent || extent > p.maxExtent)
            continue;

        // 只统计这个轮廓内部且属于前景的像素，不能平均整块黑色背景。
        cv::Mat componentMask = cv::Mat::zeros(box.size(), CV_8UC1);
        const std::vector<std::vector<cv::Point>> oneContour{contour};
        cv::drawContours(componentMask, oneContour, 0, cv::Scalar(255),
                         cv::FILLED, cv::LINE_8, cv::noArray(), 0,
                         cv::Point(-box.x, -box.y));
        cv::bitwise_and(componentMask, rawMask(box), componentMask);
        const int foreground = cv::countNonZero(componentMask);
        if (foreground == 0) continue;
        const cv::Scalar meanHSV = cv::mean(hsv(box), componentMask);
        if (meanHSV[0] > p.maxMeanHue || meanHSV[2] < p.minMeanValue)
            continue;

        // R 外侧应较独立。贴在一起的小碎片、箭头更容易被排除。
        const int pad = std::max(2, cvRound(0.4 * std::max(box.width, box.height)));
        const cv::Rect expanded = cv::Rect(
            box.x - pad, box.y - pad, box.width + 2 * pad, box.height + 2 * pad)
            & cv::Rect(0, 0, rawMask.cols, rawMask.rows);
        const double surroundings = std::max(0,
            cv::countNonZero(rawMask(expanded)) - foreground) / double(foreground);
        if (surroundings > p.maxSurroundingRatio) continue;

        const double shapeScore = std::max(0.0, 1.0 - std::abs(aspect - 1.1) / 0.6);
        const double colorScore = std::max(0.0, 1.0 - meanHSV[0] / 30.0);
        const double score = 0.45 * shapeScore + 0.35 * colorScore +
                             0.20 * (1.0 - std::min(1.0, surroundings));
        if (score < p.minShapeScore) continue;

        RCenterCandidate candidate;
        candidate.box = box;
        // box 覆盖 x 到 x+width-1 的像素中心，因此使用 (width-1)/2。
        candidate.center = cv::Point2f(
            box.x + (box.width - 1) * 0.5f,
            box.y + (box.height - 1) * 0.5f);
        candidate.contourArea = area;
        candidate.extent = extent;
        candidate.meanHue = meanHSV[0];
        candidate.meanValue = meanHSV[2];
        candidate.surroundingRatio = surroundings;
        candidate.score = score;
        candidates.push_back(candidate);
    }
    return candidates;
}

// 8. 只做单帧几何合理性检查；这个函数不进行跨帧身份关联。
bool isTargetGeometryPlausible(
    const TargetCandidate& target,
    const cv::Point2f& rCenter,
    const RCenterParameters& p)
{
    if (!std::isfinite(rCenter.x) || !std::isfinite(rCenter.y) ||
        !std::isfinite(target.center.x) || !std::isfinite(target.center.y) ||
        !std::isfinite(target.radius) || target.radius <= 0.0f)
        return false;
    const double distanceRatio = cv::norm(target.center - rCenter) / target.radius;
    return distanceRatio >= p.minDistanceRatio && distanceRatio <= p.maxDistanceRatio;
}

// 9. 从本帧候选中选择 R：有目标圆时要求至少一个几何相容目标。
// 多个候选分数太接近时，返回 lost，而不是按照轮廓序号随便挑。
RCenterResult detectRCenter(
    const cv::Mat& frame,
    const cv::Mat& rawMask,
    const std::vector<TargetCandidate>& targets,
    const RCenterParameters& p)
{
    if (frame.empty() || frame.type() != CV_8UC3 || rawMask.type() != CV_8UC1 ||
        frame.size() != rawMask.size())
        throw std::runtime_error("Invalid frame/mask for R detection.");
    validateRCenterParameters(p);
    RCenterResult result;
    result.candidates = findRCenterCandidates(frame, rawMask, p);
    if (result.candidates.empty()) return result;

    std::vector<RCenterCandidate> eligible;
    for (auto& candidate : result.candidates)
    {
        for (const auto& target : targets)
            if (isTargetGeometryPlausible(target, candidate.center, p))
                ++candidate.compatibleTargets;
        if (!targets.empty())
        {
            if (candidate.compatibleTargets == 0) continue;
            candidate.score = 0.8 * candidate.score +
                0.2 * candidate.compatibleTargets / double(targets.size());
        }
        eligible.push_back(candidate);
    }
    if (eligible.empty())
    {
        result.reason = "GEOMETRY_REJECTED";
        return result;
    }
    std::sort(eligible.begin(), eligible.end(),
        [](const RCenterCandidate& a, const RCenterCandidate& b)
        { return a.score > b.score; });
    if (eligible.size() > 1 && eligible[0].score - eligible[1].score < p.ambiguityMargin)
    {
        result.reason = "AMBIGUOUS";
        return result;
    }
    result.detected = true;
    result.center = eligible.front().center;
    result.box = eligible.front().box;
    result.score = eligible.front().score;
    result.reason = "OK";
    return result;
}

// 10. 计算相对于当前 R 的图像平面位置、距离和角度，不做速度拟合。
RelativeGeometry calculateRelativeGeometry(
    const TargetCandidate& target,
    const cv::Point2f& rCenter)
{
    if (!std::isfinite(rCenter.x) || !std::isfinite(rCenter.y) ||
        !std::isfinite(target.center.x) || !std::isfinite(target.center.y))
        throw std::runtime_error("Relative geometry requires valid centers.");
    RelativeGeometry geometry;
    geometry.imageOffset = target.center - rCenter;
    geometry.distancePx = cv::norm(geometry.imageOffset);
    if (geometry.distancePx <= 1e-6)
        throw std::runtime_error("Coincident centers: angle is undefined.");
    geometry.angleRad = std::atan2(
        -double(geometry.imageOffset.y), double(geometry.imageOffset.x));
    return geometry;
}

} // namespace task3

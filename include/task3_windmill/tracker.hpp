#pragma once

#include <limits>
#include <string>
#include <vector>

namespace task3
{
// 与图像库解耦：单元测试可以直接给跟踪器输入合成的角度和距离。
struct TrackObservation
{
    int detectionIndex = -1; // 仅用于找回“本帧”的检测结果，不是身份 ID。
    double angleRad = 0.0;
    double distancePx = 0.0; // R 到目标圆心的距离。
    double radiusPx = 0.0;   // 目标小圆自身的半径。
    double quality = 0.0;    // 只用于首次选取，不用于抢占已锁定目标。
};

enum class TrackingState { WAITING, DETECTED, LOST };

struct TrackerParameters
{
    // 第 1~7 个连续失败帧保留身份；第 8 个失败帧释放；下一帧才允许新锁定。
    int maxLostFrames = 8;
    double angleGateDeg = 20.0;
    double gateGrowthDeg = 2.0; // 每个已发生的漏检帧增加一点关联容忍。
    double maxAngleGateDeg = 30.0; // 绝不无限扩张搜索范围。
    double maxDistanceChange = 0.30;
    double maxRadiusChange = 0.35;
    double ambiguityMargin = 0.10; // 最好和次好归一化代价差过小则不强行选择。
    double motionAlpha = 0.40;
    double maxStepDeg = 7.0; // 每个“视频帧”的最大角度增量，不是物理角速度。
};

struct TrackingResult
{
    TrackingState state = TrackingState::WAITING;
    bool locked = false;         // 本次 update 后是否还保留锁定。
    bool hasObservation = false; // 只有本帧有效关联，才可以画选中圆和连线。
    int targetId = -1;
    int selectedIndex = -1;
    int lostFrames = 0;
    std::string event = "NONE";
    std::string reason = "WAITING_FOR_TARGET";
    double angleErrorRad = std::numeric_limits<double>::quiet_NaN();
};

const char* trackingStateName(TrackingState state);
double wrapToPi(double angle);

class TargetTracker
{
public:
    explicit TargetTracker(const TrackerParameters& parameters = TrackerParameters{});
    // 必须按视频帧序调用，每帧一次，包括 R/目标检测失败的帧。
    TrackingResult update(
        bool rDetected,
        const std::vector<TrackObservation>& observations);

private:
    struct Match
    {
        int position = -1; // 当前 observations 向量内的位置，不是 ID。
        double angleError = std::numeric_limits<double>::quiet_NaN();
        std::string reason = "NO_MATCH";
    };

    std::vector<TrackObservation> validObservations(
        const std::vector<TrackObservation>& observations) const;
    Match associate(const std::vector<TrackObservation>& observations) const;
    TrackingResult acquire(const std::vector<TrackObservation>& observations);
    TrackingResult accept(const TrackObservation& observation, double angleError);
    TrackingResult markLost(const std::string& reason);

    TrackerParameters parameters_;
    bool locked_ = false;
    int currentId_ = -1;
    int nextId_ = 1;
    int lostFrames_ = 0;
    long long frame_ = -1;
    long long lastObservedFrame_ = -1;
    TrackObservation last_;
    double angleStep_ = 0.0; // 只给关联用的短期参考，不作为观测输出。
    bool stepInitialized_ = false;
};
} // namespace task3

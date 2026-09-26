#include "task3_windmill/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace task3
{
namespace
{
constexpr double pi = 3.14159265358979323846;
double radians(double degrees) { return degrees * pi / 180.0; }
}

double wrapToPi(double angle)
{
    double wrapped = std::remainder(angle, 2.0 * pi);
    if (wrapped >= pi) wrapped -= 2.0 * pi;
    return wrapped;
}

const char* trackingStateName(TrackingState state)
{
    switch (state)
    {
        case TrackingState::DETECTED: return "detected";
        case TrackingState::LOST: return "lost";
        case TrackingState::WAITING: return "waiting";
    }
    return "invalid";
}

TargetTracker::TargetTracker(const TrackerParameters& p) : parameters_(p)
{
    for (double value : {p.angleGateDeg, p.gateGrowthDeg, p.maxAngleGateDeg,
                         p.maxDistanceChange, p.maxRadiusChange, p.ambiguityMargin,
                         p.motionAlpha, p.maxStepDeg})
        if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite tracker parameter.");
    if (p.maxLostFrames < 1 || p.angleGateDeg <= 0 || p.gateGrowthDeg < 0 ||
        p.maxAngleGateDeg < p.angleGateDeg || p.maxAngleGateDeg >= 180.0 ||
        p.maxDistanceChange <= 0 || p.maxRadiusChange <= 0 ||
        p.ambiguityMargin < 0 || p.motionAlpha <= 0 || p.motionAlpha > 1 ||
        p.maxStepDeg <= 0 || p.maxStepDeg >= 180.0)
        throw std::invalid_argument("Invalid tracker parameters.");
}

std::vector<TrackObservation> TargetTracker::validObservations(
    const std::vector<TrackObservation>& observations) const
{
    std::vector<TrackObservation> valid;
    for (auto o : observations)
    {
        if (o.detectionIndex < 0 || !std::isfinite(o.angleRad) ||
            !std::isfinite(o.distancePx) || o.distancePx <= 1e-6 ||
            !std::isfinite(o.radiusPx) || o.radiusPx <= 1e-6 ||
            !std::isfinite(o.quality)) continue;
        o.angleRad = wrapToPi(o.angleRad);
        valid.push_back(o);
    }
    return valid;
}

// 1. 首次锁定或旧 ID 已释放：按质量选一个，几何值用于同分时确定性排序。
// 此规则只在未锁定时运行，绝不会因为第二个目标质量更高就抢占锁定。
TrackingResult TargetTracker::acquire(const std::vector<TrackObservation>& observations)
{
    const auto best = std::min_element(observations.begin(), observations.end(),
        [](const TrackObservation& a, const TrackObservation& b)
        {
            if (a.quality != b.quality) return a.quality > b.quality;
            if (a.angleRad != b.angleRad) return a.angleRad < b.angleRad;
            if (a.distancePx != b.distancePx) return a.distancePx < b.distancePx;
            return a.radiusPx < b.radiusPx;
        });
    locked_ = true;
    currentId_ = nextId_++;
    lostFrames_ = 0;
    last_ = *best;
    lastObservedFrame_ = frame_;
    angleStep_ = 0.0;
    stepInitialized_ = false;

    TrackingResult out;
    out.state = TrackingState::DETECTED;
    out.locked = true;
    out.hasObservation = true;
    out.targetId = currentId_;
    out.selectedIndex = best->detectionIndex;
    out.event = "ACQUIRED";
    out.reason = "NEW_TRACK_SEGMENT";
    return out;
}

// 2. 只在已有 ID 时关联。当前帧的角度与短期参考比较，尺度变化也须合理。
TargetTracker::Match TargetTracker::associate(
    const std::vector<TrackObservation>& observations) const
{
    Match result;
    const double elapsed = static_cast<double>(frame_ - lastObservedFrame_);
    const double referenceAngle = wrapToPi(last_.angleRad + angleStep_ * elapsed);
    const double angleGate = radians(std::min(parameters_.maxAngleGateDeg,
        parameters_.angleGateDeg + parameters_.gateGrowthDeg * lostFrames_));
    double bestCost = std::numeric_limits<double>::infinity();
    double secondCost = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < observations.size(); ++i)
    {
        const auto& o = observations[i];
        const double angleError = std::abs(wrapToPi(o.angleRad - referenceAngle));
        const double distanceChange = std::abs(o.distancePx - last_.distancePx) / last_.distancePx;
        const double radiusChange = std::abs(o.radiusPx - last_.radiusPx) / last_.radiusPx;
        if (angleError > angleGate || distanceChange > parameters_.maxDistanceChange ||
            radiusChange > parameters_.maxRadiusChange) continue;

        const double cost = 0.80 * angleError / angleGate +
            0.10 * distanceChange / parameters_.maxDistanceChange +
            0.10 * radiusChange / parameters_.maxRadiusChange;
        if (cost < bestCost)
        {
            secondCost = bestCost;
            bestCost = cost;
            result.position = static_cast<int>(i);
            result.angleError = angleError;
        }
        else secondCost = std::min(secondCost, cost);
    }
    if (result.position < 0) return result;
    if (secondCost - bestCost <= parameters_.ambiguityMargin)
    {
        result.position = -1;
        result.reason = "AMBIGUOUS_MATCH";
        result.angleError = std::numeric_limits<double>::quiet_NaN();
        return result;
    }
    result.reason = "MATCHED";
    return result;
}

// 3. 有效观测：保持原 ID，更新短期增量参考，清零连续丢失计数。
TrackingResult TargetTracker::accept(const TrackObservation& o, double angleError)
{
    const bool recovered = lostFrames_ > 0;
    const double elapsed = static_cast<double>(frame_ - lastObservedFrame_);
    const double limit = radians(parameters_.maxStepDeg);
    const double step = std::clamp(wrapToPi(o.angleRad - last_.angleRad) / elapsed,
                                   -limit, limit);
    angleStep_ = stepInitialized_ ?
        (1.0 - parameters_.motionAlpha) * angleStep_ + parameters_.motionAlpha * step : step;
    stepInitialized_ = true;
    last_ = o;
    lastObservedFrame_ = frame_;
    lostFrames_ = 0;

    TrackingResult out;
    out.state = TrackingState::DETECTED;
    out.locked = true;
    out.hasObservation = true;
    out.targetId = currentId_;
    out.selectedIndex = o.detectionIndex;
    out.event = recovered ? "REACQUIRED" : "TRACKED";
    out.reason = "MATCHED_EXISTING_ID";
    out.angleErrorRad = angleError;
    return out;
}

// 4. 没有可信观测：保留 ID 等待，或达到上限后释放。绝不返回旧坐标作为新观测。
TrackingResult TargetTracker::markLost(const std::string& reason)
{
    ++lostFrames_;
    TrackingResult out;
    out.state = TrackingState::LOST;
    out.targetId = currentId_;
    out.lostFrames = lostFrames_;
    out.reason = reason;
    out.event = lostFrames_ == 1 ? "LOST" : "HOLD";
    if (lostFrames_ >= parameters_.maxLostFrames)
    {
        locked_ = false;
        currentId_ = -1;
        out.event = "RELEASED";
    }
    out.locked = locked_;
    return out;
}

// 5. 每一帧调用一次；失败帧同样必须调用，不能跳帧或每帧创建一个新 tracker。
TrackingResult TargetTracker::update(
    bool rDetected, const std::vector<TrackObservation>& observations)
{
    ++frame_;
    const auto valid = validObservations(observations);
    if (!locked_)
    {
        if (rDetected && !valid.empty()) return acquire(valid);
        TrackingResult out;
        out.reason = rDetected ? "WAITING_FOR_TARGET" : "R_LOST";
        return out;
    }
    if (!rDetected) return markLost("R_LOST");
    if (valid.empty()) return markLost("NO_VALID_TARGET");
    const Match match = associate(valid);
    if (match.position >= 0)
        return accept(valid[static_cast<std::size_t>(match.position)], match.angleError);
    return markLost(match.reason);
}
} // namespace task3

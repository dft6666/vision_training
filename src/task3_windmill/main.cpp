#include "task3_windmill/detector.hpp"
#include "task3_windmill/tracker.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <algorithm>
#include <initializer_list>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

// 1. 视频信息：这里只保存数据，不执行视频处理。
struct VideoInfo
{
    cv::Size frameSize;
    double fps = 0.0;
    long long reportedFrames = 0;
};

// 2. 从已经打开的视频中读取参数，不读取或跳过任何画面。
VideoInfo readVideoInfo(cv::VideoCapture& capture)
{
    VideoInfo info;

    info.frameSize.width = static_cast<int>(
        capture.get(cv::CAP_PROP_FRAME_WIDTH));
    info.frameSize.height = static_cast<int>(
        capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    info.fps = capture.get(cv::CAP_PROP_FPS);

    const double count = capture.get(cv::CAP_PROP_FRAME_COUNT);
    if (std::isfinite(count) && count > 0.0)
    {
        info.reportedFrames = std::llround(count);
    }

    if (info.frameSize.width <= 0 || info.frameSize.height <= 0)
    {
        throw std::runtime_error("Invalid video dimensions.");
    }
    if (!std::isfinite(info.fps) || info.fps <= 0.0)
    {
        throw std::runtime_error("Invalid FPS; do not guess an FPS.");
    }

    return info;
}

// 3. 打印参数。reportedFrames 是文件报告的帧数，之后还会实际计数。
void printVideoInfo(const VideoInfo& info)
{
    std::cout << "Resolution: "
              << info.frameSize.width << " x "
              << info.frameSize.height << '\n';
    std::cout << std::fixed << std::setprecision(3)
              << "FPS: " << info.fps << '\n';

    if (info.reportedFrames > 0)
    {
        std::cout << "Reported frames: " << info.reportedFrames << '\n';
        std::cout << "Estimated duration: "
                  << info.reportedFrames / info.fps << " s\n";
    }
    else
    {
        std::cout << "Reported frames: unknown; will count while reading.\n";
    }
}

// 4. 创建输出视频。保持输入尺寸和 FPS，使用 mp4v 编码彩色帧。
void openOutputVideo(
    cv::VideoWriter& writer,
    const fs::path& outputPath,
    const VideoInfo& info)
{
    const int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

    writer.open(
        outputPath.string(),
        codec,
        info.fps,
        info.frameSize,
        true
    );

    if (!writer.isOpened())
    {
        throw std::runtime_error(
            "Cannot open output video: " + outputPath.string());
    }
}


// 5. 将 M5 单帧结果适配成跟踪器所需的纯数值观测，不改动检测阈值。
std::vector<task3::TrackObservation> makeTrackObservations(
    const std::vector<task3::TargetCandidate>& targets,
    const task3::RCenterResult& r, const task3::RCenterParameters& parameters)
{
    std::vector<task3::TrackObservation> observations;
    if (!r.detected) return observations;
    for (std::size_t i = 0; i < targets.size(); ++i)
    {
        const auto& target = targets[i];
        if (!task3::isTargetGeometryPlausible(target, r.center, parameters)) continue;
        const auto geometry = task3::calculateRelativeGeometry(target, r.center);
        observations.push_back({static_cast<int>(i), geometry.angleRad,
            geometry.distancePx, target.radius, target.confidence});
    }
    return observations;
}

// 6. 本帧选中结果的访问只能在 hasObservation=true 时进行。
const task3::TargetCandidate* selectedTarget(
    const std::vector<task3::TargetCandidate>& targets, const task3::TrackingResult& track)
{
    if (!track.hasObservation) return nullptr;
    if (track.selectedIndex < 0 || static_cast<std::size_t>(track.selectedIndex) >= targets.size())
        throw std::runtime_error("Tracker returned an invalid current-frame index.");
    return &targets[static_cast<std::size_t>(track.selectedIndex)];
}

// 7. 灰色圆=其他候选；绿色粗圆=当前有效锁定；黄色线=当前 R 到选中目标。
// lost 时保留文本 ID，但不画旧圆、旧连线或推算位置。R 可独立显示。
cv::Mat drawTrackingOverlay(
    const cv::Mat& frame, const std::vector<task3::TargetCandidate>& targets,
    const task3::RCenterResult& r, const task3::TrackingResult& track, long long index)
{
    cv::Mat overlay = frame.clone();
    for (const auto& t : targets)
        cv::circle(overlay, cv::Point(cvRound(t.center.x), cvRound(t.center.y)),
                   cvRound(t.radius), cv::Scalar(140,140,140), 1, cv::LINE_AA);
    if (r.detected)
    {
        cv::rectangle(overlay, r.box, cv::Scalar(255,0,255), 1, cv::LINE_AA);
        cv::drawMarker(overlay, r.center, cv::Scalar(255,0,255), cv::MARKER_CROSS, 14, 2);
        cv::putText(overlay, "R", cv::Point(r.box.x, std::max(120, r.box.y-7)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,255), 2, cv::LINE_AA);
    }
    const auto* selected = selectedTarget(targets, track);
    if (selected)
    {
        if (!r.detected) throw std::runtime_error("Selected observation requires a valid R.");
        const cv::Point center(cvRound(selected->center.x), cvRound(selected->center.y));
        cv::circle(overlay, center, cvRound(selected->radius), cv::Scalar(0,255,0), 3, cv::LINE_AA);
        cv::circle(overlay, center, 4, cv::Scalar(255,255,0), cv::FILLED, cv::LINE_AA);
        cv::line(overlay, r.center, center, cv::Scalar(0,255,255), 2, cv::LINE_AA);
        const auto g = task3::calculateRelativeGeometry(*selected, r.center);
        const std::string label = "ID=" + std::to_string(track.targetId) +
            cv::format(" theta=%.1f deg", g.angleRad * 180.0 / CV_PI);
        cv::putText(overlay, label,
            cv::Point(std::clamp(center.x-50, 0, std::max(0, frame.cols-300)),
                      std::clamp(center.y-cvRound(selected->radius)-12, 120, std::max(120,frame.rows-10))),
            cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0,255,0), 2, cv::LINE_AA);
    }
    // WAITING 仍明确标明没有有效观测，而不是画面看似成功。
    const std::string state = track.state == task3::TrackingState::WAITING
        ? "lost (waiting)" : task3::trackingStateName(track.state);
    const std::string id = track.targetId < 0 ? "none" : std::to_string(track.targetId);
    cv::rectangle(overlay, cv::Rect(0,0,std::min(overlay.cols,1150),std::min(overlay.rows,105)),
                  cv::Scalar(0,0,0), cv::FILLED);
    const cv::Scalar stateColor = track.hasObservation ? cv::Scalar(0,255,0) : cv::Scalar(0,190,255);
    cv::putText(overlay, "M6 frame=" + std::to_string(index) + " ID=" + id +
                " state=" + state + " event=" + track.event,
                cv::Point(15,28), cv::FONT_HERSHEY_SIMPLEX, 0.65, stateColor, 2, cv::LINE_AA);
    cv::putText(overlay, "R=" + std::string(r.detected?"detected":"lost") +
                " reason=" + track.reason + " lost_frames=" + std::to_string(track.lostFrames) +
                " lock=" + (track.locked?"held":"none"),
                cv::Point(15,57), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 1, cv::LINE_AA);
    cv::putText(overlay, "Gray=candidate  Green=selected observation  Yellow=current R-to-target",
                cv::Point(15,85), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(210,210,210), 1, cv::LINE_AA);
    return overlay;
}

struct RunStats
{
    long long frames = 0, detected = 0, lost = 0, waiting = 0, rDetected = 0;
    long long acquisitions = 0, releases = 0, recoveries = 0, multiCandidateFrames = 0;
};

bool isKeyEvent(const task3::TrackingResult& r)
{
    return r.event == "ACQUIRED" || r.event == "REACQUIRED" ||
           r.event == "LOST" || r.event == "RELEASED";
}

// 8. 固定列数，每帧恰好一行。没有有效关联时 target/angle 相关字段留空。
void writeTrackingRow(std::ostream& csv, long long frame, const VideoInfo& info,
    std::size_t validCount, const std::vector<task3::TargetCandidate>& targets,
    const task3::RCenterResult& r, const task3::TrackingResult& track)
{
    const auto num=[](double value) {
        std::ostringstream s; s << std::fixed << std::setprecision(6) << value; return s.str(); };
    std::vector<std::string> cells(23);
    cells[0]=std::to_string(frame); cells[1]=num(frame/info.fps); // 仅播放时刻。
    cells[2]=r.detected?"detected":"lost"; cells[3]=r.reason;
    cells[4]=std::to_string(validCount);
    if(track.targetId>=0) cells[5]=std::to_string(track.targetId);
    cells[6]=task3::trackingStateName(track.state); cells[7]=track.event; cells[8]=track.reason;
    cells[9]=std::to_string(track.lostFrames); cells[10]=track.locked?"1":"0";
    if(r.detected) { cells[12]=num(r.center.x); cells[13]=num(r.center.y); }
    if(const auto* t=selectedTarget(targets,track))
    {
        const auto g=task3::calculateRelativeGeometry(*t,r.center);
        cells[11]=std::to_string(track.selectedIndex);
        cells[14]=num(t->center.x); cells[15]=num(t->center.y); cells[16]=num(t->radius);
        cells[17]=num(g.imageOffset.x); cells[18]=num(g.imageOffset.y);
        cells[19]=num(g.distancePx); cells[20]=num(g.angleRad); cells[21]=num(g.angleRad*180/CV_PI);
        if(std::isfinite(track.angleErrorRad)) cells[22]=num(track.angleErrorRad*180/CV_PI);
    }
    for(std::size_t i=0;i<cells.size();++i) { if(i) csv << ','; csv << cells[i]; }
    csv << '\n';
    if(!csv) throw std::runtime_error("Cannot write tracking CSV.");
}

// 9. 新建 debug_m6，不覆盖前几个阶段的调试图片。
void saveTrackingDebug(const cv::Mat& frame, const cv::Mat& rawMask, const cv::Mat& overlay,
                       const fs::path& outputDir, long long index)
{
    const fs::path dir=outputDir/"debug_m6"; fs::create_directories(dir);
    const std::string name="frame_"+std::to_string(index);
    if(!cv::imwrite((dir/(name+"_original.png")).string(),frame) ||
       !cv::imwrite((dir/(name+"_raw.png")).string(),rawMask) ||
       !cv::imwrite((dir/(name+"_tracking.png")).string(),overlay))
        throw std::runtime_error("Cannot save tracking debug images.");
}

// 10. 一个视频只创建一次 tracker；所有帧都调用 update 和两个 writer。
RunStats processAllFrames(cv::VideoCapture& capture, cv::VideoWriter& binaryWriter,
    cv::VideoWriter& overlayWriter, const VideoInfo& info,
    const task3::MaskParameters& maskParameters,
    const task3::CircleParameters& circleParameters,
    const task3::RCenterParameters& rParameters,
    const task3::TrackerParameters& trackerParameters, const fs::path& outputDir)
{
    std::ofstream csv(outputDir/"tracking.csv"), events(outputDir/"tracking_events.csv");
    if(!csv || !events) throw std::runtime_error("Cannot open tracking logs.");
    const std::string header="frame,video_time_s,r_state,r_reason,valid_candidates,target_id,"
        "track_state,event,reason,lost_frames,lock_active,selected_index,r_x,r_y,"
        "target_x,target_y,target_radius,dx_img,dy_img,distance_px,angle_rad,angle_deg,angle_error_deg\n";
    csv << header; events << header;
    task3::TargetTracker tracker(trackerParameters); // 关键：在 while 外创建。
    cv::Mat frame; RunStats stats;
    while(capture.read(frame))
    {
        if(frame.empty() || frame.size()!=info.frameSize || frame.type()!=CV_8UC3)
            throw std::runtime_error("Invalid input frame.");
        const auto raw=task3::createOrangeMask(frame,maskParameters);
        const auto clean=task3::cleanOrangeMask(raw,maskParameters);
        const auto targets=task3::detectTargetCandidates(raw,clean,circleParameters);
        const auto r=task3::detectRCenter(frame,raw,targets,rParameters);
        const auto observations=makeTrackObservations(targets,r,rParameters);
        const auto track=tracker.update(r.detected,observations);
        const auto overlay=drawTrackingOverlay(frame,targets,r,track,stats.frames);
        writeTrackingRow(csv,stats.frames,info,observations.size(),targets,r,track);
        if(isKeyEvent(track)) writeTrackingRow(events,stats.frames,info,observations.size(),targets,r,track);
        if(stats.frames%150==0 || isKeyEvent(track))
            saveTrackingDebug(frame,raw,overlay,outputDir,stats.frames);
        cv::Mat binaryBgr; cv::cvtColor(clean,binaryBgr,cv::COLOR_GRAY2BGR);
        binaryWriter.write(binaryBgr); overlayWriter.write(overlay); // lost 帧也完整写出。
        ++stats.frames;
        if(r.detected) ++stats.rDetected;
        if(observations.size()>=2) ++stats.multiCandidateFrames;
        if(track.state==task3::TrackingState::DETECTED) ++stats.detected;
        else if(track.state==task3::TrackingState::LOST) ++stats.lost;
        else ++stats.waiting;
        if(track.event=="ACQUIRED") ++stats.acquisitions;
        if(track.event=="REACQUIRED") ++stats.recoveries;
        if(track.event=="RELEASED") ++stats.releases;
        if(stats.frames%100==0)
            std::cout << "Processed " << stats.frames << " ID=" << track.targetId
                      << " state=" << task3::trackingStateName(track.state)
                      << " event=" << track.event << std::endl;
    }
    if(stats.frames==0) throw std::runtime_error("No input frames decoded.");
    csv.close(); events.close();
    if(!csv || !events) throw std::runtime_error("Cannot finish tracking logs.");
    return stats;
}
// 6. 重新打开输出并实际解码计数，检查保存后是否仍能完整读取。
void verifyOutputVideo(
    const fs::path& outputPath,
    const VideoInfo& inputInfo,
    long long expectedFrames)
{
    cv::VideoCapture check(outputPath.string());
    if (!check.isOpened())
    {
        throw std::runtime_error("Cannot reopen the output video.");
    }

    const VideoInfo outputInfo = readVideoInfo(check);
    if (outputInfo.frameSize != inputInfo.frameSize ||
        std::abs(outputInfo.fps - inputInfo.fps) > 0.001)
    {
        throw std::runtime_error("Output dimensions or FPS do not match.");
    }

    cv::Mat frame;
    long long decodedFrames = 0;
    while (check.read(frame))
    {
        if (frame.empty() || frame.size() != inputInfo.frameSize)
        {
            throw std::runtime_error("Invalid frame in the output video.");
        }
        ++decodedFrames;
    }

    std::cout << "Output frames decoded: " << decodedFrames << '\n';
    if (decodedFrames != expectedFrames)
    {
        throw std::runtime_error("Output frame count does not match input.");
    }

    std::cout << "CHECK PASSED: size, FPS and frame count match.\n";
}

// 12. 本机真实运行统计，不将“返回 detected 的数量”称作检测准确率。
void writeRunSummary(const fs::path& dir, const std::string& input,
    const VideoInfo& info, const RunStats& s, const task3::TrackerParameters& p,
    const task3::MaskParameters& m, const task3::CircleParameters& c,
    const task3::RCenterParameters& r)
{
    std::ofstream out(dir/"tracking_summary.md");
    if(!out) throw std::runtime_error("Cannot open summary.");
    out << "# 本机 Milestone 6 运行记录\n\n输入：`" << input << "`\n\n"
        << "OpenCV：" << CV_VERSION << "；尺寸：" << info.frameSize.width << "×"
        << info.frameSize.height << "；FPS：" << info.fps << "；完整处理帧数：" << s.frames << "。\n\n"
        << "两个输出视频均已重新解码检查尺寸、FPS 和帧数。该检查不代表检测或身份全部正确。\n\n"
        << "| 统计 | 数量 |\n|---|---:|\n"
        << "| detected | " << s.detected << " |\n| lost | " << s.lost
        << " |\n| waiting | " << s.waiting << " |\n| R detected | " << s.rDetected
        << " |\n| 至少两个有效候选的帧 | " << s.multiCandidateFrames
        << " |\n| 新建 ID / ACQUIRED | " << s.acquisitions
        << " |\n| 同 ID 恢复 / REACQUIRED | " << s.recoveries
        << " |\n| 释放 ID / RELEASED | " << s.releases << " |\n\n"
        << "## 实际参数\n\n"
        << "HSV H=["<<m.hMin<<","<<m.hMax<<"], S>="<<m.sMin<<", V>="<<m.vMin
        << "; open="<<m.openKernel<<", close="<<m.closeKernel<<"。\n\n"
        << "Circle radius=["<<c.minRadius<<","<<c.maxRadius<<"], minCenterDistance="<<c.minCenterDistance
        << ", Hough="<<c.houghThreshold<<", innerRatio="<<c.innerRadiusRatio
        << ", gapRatio="<<c.gapRadiusRatio<<", outer>="<<c.minOuterSupport
        << ", inner>="<<c.minInnerSupport<<", gap<="<<c.maxGapSupport<<"。\n\n"
        << "R area=["<<r.minArea<<","<<r.maxArea<<"], width=["<<r.minWidth<<","<<r.maxWidth
        << "], height=["<<r.minHeight<<","<<r.maxHeight<<"], aspect=["<<r.minAspect<<","<<r.maxAspect
        << "], extent=["<<r.minExtent<<","<<r.maxExtent<<"], H<="<<r.maxMeanHue<<", V>="<<r.minMeanValue
        << ", surround<="<<r.maxSurroundingRatio<<", score>="<<r.minShapeScore
        << ", ambiguity="<<r.ambiguityMargin<<", distanceRatio=["<<r.minDistanceRatio<<","<<r.maxDistanceRatio<<"]。\n\n"
        << "Tracker maxLostFrames="<<p.maxLostFrames<<", angleGateDeg="<<p.angleGateDeg
        << ", gateGrowthDeg="<<p.gateGrowthDeg<<", maxAngleGateDeg="<<p.maxAngleGateDeg
        << ", maxDistanceChange="<<p.maxDistanceChange<<", maxRadiusChange="<<p.maxRadiusChange
        << ", ambiguityMargin="<<p.ambiguityMargin<<", motionAlpha="<<p.motionAlpha
        << ", maxStepDeg="<<p.maxStepDeg<<"。\n\n"
        << "## 身份与失败规则\n\n首次任选一个有效候选，本实现按质量选择；已锁定后只按关联规则更新。"
        << "第 "<<p.maxLostFrames<<" 个连续失败帧显示 RELEASED，下一帧才允许新建 ID。"
        << "短暂丢失恢复为 REACQUIRED，沿用旧 ID；释放后的 ACQUIRED 使用新 ID。"
        << "ID 表示一次连续锁定片段，不是永久扇叶编号；每个视频从 1 开始。\n\n"
        << "R 失效同样计为无有效观测。没有被匹配的目标坐标/角度留空，不绘制旧结果。"
        << "角度增量仅用于图像序列关联，不是现场角速度；video_time_s 是播放时间。\n\n"
        << "## 局限与人工验收\n\nM4/M5 的漏检、误检仍会传递到跟踪器。"
        << "强透视、突变运动、长时间丢失后相似目标占据相同方位均可能使本方法失败。"
        << "没有人工逐帧真值，以上不是准确率、召回率或 ID 切换率。"
        << "请对照 tracking_events.csv 检查每次 ACQUIRED/RELEASED，并在总报告中补充误检例子。\n\n"
        << "输出：[标注视频](recognition_overlay.mp4)、[二值化](binary_process.mp4)、"
        << "[逐帧日志](tracking.csv)、[事件日志](tracking_events.csv)。\n";
    out.close(); if(!out) throw std::runtime_error("Cannot finish run summary.");
}

// 13. 沿用相同输入/输出参数；不修改原始视频，不改旧的 center_overlay.mp4。
int processVideo(const std::string& inputPath, const std::string& outputDir)
{
    if(!fs::is_regular_file(inputPath)) throw std::runtime_error("Input file not found: "+inputPath);
    cv::VideoCapture capture(inputPath);
    if(!capture.isOpened()) throw std::runtime_error("Cannot open input video.");
    const auto info=readVideoInfo(capture); printVideoInfo(info);
    fs::create_directories(outputDir);
    const fs::path binary=fs::path(outputDir)/"binary_process.mp4";
    const fs::path overlay=fs::path(outputDir)/"recognition_overlay.mp4";
    for(const auto& path:{binary,overlay})
        if(fs::exists(path) && fs::equivalent(inputPath,path))
            throw std::runtime_error("Input and output must differ.");
    if(fs::exists(binary) && fs::exists(overlay) && fs::equivalent(binary,overlay))
        throw std::runtime_error("Outputs must not be aliases of the same file.");
    cv::VideoWriter binaryWriter,overlayWriter;
    openOutputVideo(binaryWriter,binary,info); openOutputVideo(overlayWriter,overlay,info);
    const task3::MaskParameters m;
    const task3::CircleParameters c;
    const task3::RCenterParameters r;
    const task3::TrackerParameters p;
    std::cout << "M6: maxLostFrames=" << p.maxLostFrames << " angleGate=" << p.angleGateDeg
              << ".." << p.maxAngleGateDeg << " deg\n";
    const auto stats=processAllFrames(capture,binaryWriter,overlayWriter,info,m,c,r,p,outputDir);
    binaryWriter.release(); overlayWriter.release(); capture.release();
    if(info.reportedFrames>0 && stats.frames!=info.reportedFrames)
        throw std::runtime_error("Input frame count mismatch; check decoding.");
    std::cout << "Input frames read: " << stats.frames << '\n';
    verifyOutputVideo(binary,info,stats.frames); verifyOutputVideo(overlay,info,stats.frames);
    writeRunSummary(outputDir,inputPath,info,stats,p,m,c,r);
    std::cout << "Saved: " << overlay.string() << '\n';
    return 0;
}

// 14. main 只做入口和异常报告。
int main(int argc,char* argv[])
{
    if(argc!=3)
    {
        std::cerr << "Usage: " << argv[0] << " <input_video> <output_directory>\n";
        return 1;
    }
    try { return processVideo(argv[1],argv[2]); }
    catch(const std::exception& error)
    { std::cerr << "ERROR: " << error.what() << '\n'; return 1; }
}

#pragma once

#include <cstddef>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "cross_state.hpp"
#include "ring_state.hpp"
//本文件中定义了选择中线的函数和类，包括根据十字路口状态和环形赛道状态选择中线，以及计算前方中线评分的函数
namespace bird {

struct CenterLineSelection {
    const std::vector<cv::Point2f>* line = nullptr;
    const char* side = "NONE";
};

inline bool sampleEdgeAtY(const std::vector<cv::Point2f>& edge, float y, float& x)
{
    for (std::size_t i = 1; i < edge.size(); ++i) {
        const auto& a = edge[i - 1];
        const auto& b = edge[i];
        if (a.y >= y && b.y < y) {
            x = a.x + (b.x - a.x) * (y - a.y) / (b.y - a.y);
            return true;
        }
    }
    return false;
}

inline cv::Point2f straightRoadPreview(const std::vector<cv::Point2f>& left,
                                       const std::vector<cv::Point2f>& right,
                                       const cv::Point2f& vehicle, const cv::Point2f& near,
                                       float nearWidth)
{
    if (!hasStraightCorridor(left, right)) return near;
    const float farY = vehicle.y - config::kStraightLookahead;
    float midX, farX;
    const auto centerAt = [&](float y, float& center) {
        float l, r;
        if (!sampleEdgeAtY(left, y, l) || !sampleEdgeAtY(right, y, r)) return false;
        const float width_m = (r - l) * config::kMetersPerPixel;
        if (width_m <= 0.22f || width_m >= 0.72f ||
            std::abs((r - l - nearWidth) * config::kMetersPerPixel) >= 0.06f) return false;
        center = (l + r) * 0.5f;
        return true;
    };
    /* Keep the short target on curves or without two measured boundaries.
     * The longer target retains the measured lateral offset and heading. */
    if (!centerAt((near.y + farY) * 0.5f, midX) || !centerAt(farY, farX) ||
        std::abs(near.x + farX - 2.f * midX) * config::kMetersPerPixel > 0.008f)
        return near;
    return {farX, farY};
}

// 给候选中线评分：优先考虑向前延伸较远、横向跳变较小的那条线。
inline float forwardLineScore(
    const std::vector<cv::Point2f>& line,
    const cv::Point2f& vehiclePoint)
{
    if (line.size() < 2) {
        return 1e9f;
    }

    cv::Point2f farPoint = line.front();

    for (const cv::Point2f& p : line) {
        if (p.y < farPoint.y) {
            farPoint = p;
        }
    }

    const float forwardDistance = vehiclePoint.y - farPoint.y;

    if (forwardDistance < 30.f) {
        return 1e9f;
    }

    const float lateralDistance =
        std::abs(farPoint.x - vehiclePoint.x);

    return lateralDistance - forwardDistance * 0.05f;
}
// 普通/十字场景选线阶段：十字内比较左右候选，普通路段优先使用有效边线推算中线。
inline CenterLineSelection selectCenterLine(
    CrossState state,
    const std::vector<cv::Point2f>& leftCenter,
    const std::vector<cv::Point2f>& rightCenter,
    const cv::Point2f& vehiclePoint,
    std::size_t leftCount,
    std::size_t rightCount)
{
    const bool leftValid =
        leftCount >= 2 && !leftCenter.empty();

    const bool rightValid =
        rightCount >= 2 && !rightCenter.empty();

    CenterLineSelection result;

    if (state == CrossState::In) {
        const float leftScore = leftValid
            ? forwardLineScore(leftCenter, vehiclePoint) : 1e9f;

        const float rightScore = rightValid? forwardLineScore(rightCenter, vehiclePoint) : 1e9f;
        if (leftScore <= rightScore && leftScore < 1e9f)
        {
            result.line = &leftCenter;
            result.side = "LEFT";
        } 
        else if (rightScore < 1e9f) 
        {
            result.line = &rightCenter;
            result.side = "RIGHT";
        }
    }
     else {
        // CROSS_NONE / CROSS_BEGIN：优先使用左侧推算的中线。
        if (leftValid) {
            result.line = &leftCenter;
            result.side = "LEFT";
        } else if (rightValid) {
            result.line = &rightCenter;
            result.side = "RIGHT";
        }
    }

    return result;
}
//选择中线函数
inline CenterLineSelection selectRingCenter(RingState state,
    const std::vector<cv::Point2f>& leftCenter,const std::vector<cv::Point2f>& rightCenter)
{
    CenterLineSelection result;

    const bool useLeft =state == RingState::LeftIn ||state == RingState::LeftRunning ||
        state == RingState::LeftOut ||state == RingState::RightBegin ||state == RingState::RightEnd;

    const bool useRight =state == RingState::LeftBegin ||state == RingState::LeftEnd ||
        state == RingState::RightIn ||state == RingState::RightRunning ||state == RingState::RightOut;

    if (useLeft && !leftCenter.empty()) {
        result.line = &leftCenter;
        result.side = "LEFT_RING";
    } else if (useRight && !rightCenter.empty()) {
        result.line = &rightCenter;
        result.side = "RIGHT_RING";
    }

    return result;
}
// 跨帧选线稳定器：只有另一候选持续明显更优若干帧才切换，降低左右抖动。
class CenterLineSelector {
public:
    CenterLineSelection update(CrossState state, const std::vector<cv::Point2f>& leftCenter,
        const std::vector<cv::Point2f>& rightCenter, const cv::Point2f& vehiclePoint,
        std::size_t leftCount, std::size_t rightCount)
    {
        if (state == CrossState::None) {
            side_ = 0;
            pendingFrames_ = 0;

            return selectCenterLine(state, leftCenter, rightCenter,
                vehiclePoint, leftCount, rightCount);
        }
        const float leftScore = leftCount >= 2
            ? forwardLineScore(leftCenter, vehiclePoint) : 1e9f;
        const float rightScore = rightCount >= 2
            ? forwardLineScore(rightCenter, vehiclePoint) : 1e9f;

        const bool leftValid = leftScore < 1e9f;
        const bool rightValid = rightScore < 1e9f;

        if (!leftValid && !rightValid) 
        {
            side_ = 0;
            pendingFrames_ = 0;
            return {};
        }

        if (!leftValid || !rightValid) {
            side_ = leftValid ? -1 : 1;
            pendingFrames_ = 0;
        } else if (side_ == 0) {
            side_ = leftScore <= rightScore ? -1 : 1;
            pendingFrames_ = 0;
        } else {
            const float currentScore = side_ == -1 ? leftScore : rightScore;
            const float otherScore = side_ == -1 ? rightScore : leftScore;

            if (otherScore + switchMargin_ < currentScore) {
                ++pendingFrames_;
                if (pendingFrames_ >= switchFrames_) {
                    side_ = -side_;
                    pendingFrames_ = 0;
                }
            } else {
                pendingFrames_ = 0;
            }
        }

        CenterLineSelection result;
        result.line = side_ == -1 ? &leftCenter : &rightCenter;
        result.side = side_ == -1 ? "LEFT" : "RIGHT";
        return result;
    }

private:
    int side_ = 0;           // 0：未选择，-1：左，1：右
    int pendingFrames_ = 0;  // 另一侧连续明显更好的帧数
    static constexpr float switchMargin_ = 10.f;
    static constexpr int switchFrames_ = 3;
};
} // namespace bird

#pragma once

#include <opencv2/opencv.hpp>
#include <cstddef>
#include "bird_view.hpp"
#include <algorithm>
namespace bird {

enum class RingState {
    None,
    LeftBegin,
    RightBegin,
    LeftIn,
    RightIn,
    LeftRunning,
    RightRunning,
    LeftOut,
    RightOut,
    LeftEnd,
    RightEnd
};
// 环岛识别前置检查：统计边线长度与转角，判断一侧是否呈长直道特征。
inline bool isLongStraight(const std::vector<cv::Point2f>& line)
{
    if (line.size() < 40) return false;

    const int window = 4;
    const std::size_t end = std::min(line.size() - window, std::size_t(40));
    float angleSum = 0.f;
    int count = 0, sharpCount = 0;

    for (std::size_t i = window; i < end; ++i) {
        const float angle = turnAngleDegrees(line, i, window);
        angleSum += angle;
        ++count;
        if (angle > 20.f) ++sharpCount;
    }

    return count >= 20 && angleSum / count <= 12.f && sharpCount <= count / 5;
}

inline bool hasStraightCorridor(const std::vector<cv::Point2f>& left,
                               const std::vector<cv::Point2f>& right)
{
    if (!isLongStraight(left) || !isLongStraight(right)) return false;

    const auto leftSpan = left[20] - left.front();
    const auto rightSpan = right[20] - right.front();
    if (leftSpan.y >= -60.f || rightSpan.y >= -60.f ||
        std::abs(leftSpan.x / leftSpan.y - rightSpan.x / rightSpan.y) > 0.16f)
        return false;

    const auto across = cv::Point2f(-leftSpan.y, leftSpan.x) *
                        (config::kMetersPerPixel / static_cast<float>(cv::norm(leftSpan)));
    const float nearWidth = (right.front() - left.front()).dot(across);
    const float farWidth = (right[20] - left[20]).dot(across);
    return nearWidth > 0.22f && nearWidth < 0.72f && std::abs(farWidth - nearWidth) < 0.06f;
}
// 入口识别阶段：一侧出现明显角点、另一侧保持长直线时，提出对应环岛候选。
inline bool detectLeftRingBegin(
    const std::vector<cv::Point2f>& leftLine,
    const std::vector<cv::Point2f>& rightLine,
    cv::Point2f& corner)
{
    float angle = 0.f;
    cv::Point2f otherCorner;
    const bool leftCorner = findCorner(leftLine, corner, angle);
    const bool rightCorner = findCorner(rightLine, otherCorner, angle);
    return leftCorner && !rightCorner && isLongStraight(rightLine) &&
           !hasStraightCorridor(leftLine, rightLine);
}

inline bool detectRightRingBegin(
    const std::vector<cv::Point2f>& leftLine,
    const std::vector<cv::Point2f>& rightLine,
    cv::Point2f& corner)
{
    float angle = 0.f;
    cv::Point2f otherCorner;
    const bool rightCorner = findCorner(rightLine, corner, angle);
    const bool leftCorner = findCorner(leftLine, otherCorner, angle);
    return rightCorner && !leftCorner && isLongStraight(leftLine) &&
           !hasStraightCorridor(leftLine, rightLine);
}

// 环岛通过阶段机：入口确认 -> 入环 -> 环内运行 -> 出环 -> 直道恢复后退出。
struct RingStateMachine {
    RingState state = RingState::None;
    int lostFrames = 0;
    int recoveredFrames = 0;
    int turnFrames = 0;
    int outFrames = 0;
    int entryFrames = 0;
    int entrySide = 0;
    int straightFrames = 0;
    RingState update(bool leftBegin, bool rightBegin, bool leftCornerFound, bool rightCornerFound,
    const std::vector<cv::Point2f>& leftLine, const std::vector<cv::Point2f>& rightLine)
    {
        const std::size_t leftCount = leftLine.size(), rightCount = rightLine.size();
        const bool leftLost = leftCount < 20, rightLost = rightCount < 20;

        // A sustained ordinary corridor can disprove an early entry, before any confirmed turn.
        const bool entering = state == RingState::LeftBegin || state == RingState::RightBegin ||
                              state == RingState::LeftIn || state == RingState::RightIn;
        const bool straight = entering && hasStraightCorridor(leftLine, rightLine);
        if (straight) {
            // A straight corridor can be diagonal when the car is angled.
            // Its outer edge alone is not evidence of turning into a ring.
            lostFrames = recoveredFrames = turnFrames = 0;
            if (++straightFrames >= 6) *this = RingStateMachine{};
            return state;
        }
        straightFrames = 0;

        // 点序应从近到远；比较近端约 50 像素弧长处的横向变化。
        const bool rightBendsLeft = rightCount > 10 &&
            rightLine[10].y < rightLine[0].y - 20.f && rightLine[10].x < rightLine[0].x - 10.f;
        const bool leftBendsRight = leftCount > 10 &&
            leftLine[10].y < leftLine[0].y - 20.f && leftLine[10].x > leftLine[0].x + 10.f;

        cv::Point2f leftExit, rightExit;
        float leftAngle = 0.f, rightAngle = 0.f;
        const bool leftExitNear = leftCornerFound && findCorner(leftLine, leftExit, leftAngle)
            && cv::norm(leftExit - leftLine.front()) < 130.f;
        const bool rightExitNear = rightCornerFound && findCorner(rightLine, rightExit, rightAngle)
            && cv::norm(rightExit - rightLine.front()) < 130.f;

        switch (state) {
        case RingState::None:
            lostFrames = recoveredFrames = turnFrames = outFrames = 0;
            {
                const int candidate = leftBegin && !rightBegin ? -1 : (rightBegin && !leftBegin ? 1 : 0);
                entryFrames = candidate != 0 && candidate == entrySide ? entryFrames + 1 : (candidate != 0 ? 1 : 0);
                entrySide = candidate;
                if (entryFrames >= 4) {
                    state = candidate == -1 ? RingState::LeftBegin : RingState::RightBegin;
                    entryFrames = entrySide = 0;
                }
            }
            break;

        case RingState::LeftBegin:
            if (leftLost) {
                if (lostFrames < 2) ++lostFrames;
                recoveredFrames = 0;
            } else if (lostFrames >= 2 && leftCount >= 40) {
                if (++recoveredFrames >= 2) {
                    state = RingState::LeftIn;
                    lostFrames = recoveredFrames = 0;
                }
            } else {
                recoveredFrames = 0;
                if (lostFrames < 2) lostFrames = 0;
            }
            break;

        case RingState::RightBegin:
            if (rightLost) {
                if (lostFrames < 2) ++lostFrames;
                recoveredFrames = 0;
            } else if (lostFrames >= 2 && rightCount >= 40) {
                if (++recoveredFrames >= 2) {
                    state = RingState::RightIn;
                    lostFrames = recoveredFrames = 0;
                }
            } else {
                recoveredFrames = 0;
                if (lostFrames < 2) lostFrames = 0;
            }
            break;

        case RingState::LeftIn:
            turnFrames = rightBendsLeft ? turnFrames + 1 : 0;
            if (turnFrames >= 3) {
                state = RingState::LeftRunning;
                turnFrames = 0;
            }
            break;

        case RingState::RightIn:
            turnFrames = leftBendsRight ? turnFrames + 1 : 0;
            if (turnFrames >= 3) {
                state = RingState::RightRunning;
                turnFrames = 0;
            }
            break;

        case RingState::LeftRunning:
            outFrames = rightExitNear ? outFrames + 1 : 0;
            if (outFrames >= 2) {
                state = RingState::LeftOut;
                outFrames = 0;
            }
            break;

        case RingState::RightRunning:
            outFrames = leftExitNear ? outFrames + 1 : 0;
            if (outFrames >= 2) {
                state = RingState::RightOut;
                outFrames = 0;
            }
            break;

        case RingState::LeftOut:
            recoveredFrames = isLongStraight(rightLine) ? recoveredFrames + 1 : 0;
            if (recoveredFrames >= 2) {
                state = RingState::LeftEnd;
                recoveredFrames = 0;
            }
            break;

        case RingState::RightOut:
            recoveredFrames = isLongStraight(leftLine) ? recoveredFrames + 1 : 0;
            if (recoveredFrames >= 2) {
                state = RingState::RightEnd;
                recoveredFrames = 0;
            }
            break;

        case RingState::LeftEnd:
        case RingState::RightEnd:
            // End 阶段继续沿外侧直线，内侧必须先丢失再恢复才能退出。
            {
            const bool innerLost = state == RingState::LeftEnd ? leftLost : rightLost;
            const bool innerRecovered = (state == RingState::LeftEnd ? leftCount : rightCount) >= 40;
            const bool outerStraight = isLongStraight(state == RingState::LeftEnd ? rightLine : leftLine);
            if (innerLost) {
                if (lostFrames < 2) ++lostFrames;
                recoveredFrames = 0;
            } else if (lostFrames >= 2 && innerRecovered && outerStraight) {
                ++recoveredFrames;
            } else {
                recoveredFrames = 0;
                if (lostFrames < 2) lostFrames = 0;
            }
            if (recoveredFrames >= 3) {
                state = RingState::None;
                lostFrames = recoveredFrames = turnFrames = outFrames = 0;
            }
            }
            break;
        }

        return state;
    }
};

inline const char* ringStateName(RingState state)
{
    switch (state) {
    case RingState::None: return "RING_NONE";
    case RingState::LeftBegin: return "LEFT_BEGIN";
    case RingState::RightBegin: return "RIGHT_BEGIN";
    case RingState::LeftIn: return "LEFT_IN";
    case RingState::RightIn: return "RIGHT_IN";
    case RingState::LeftRunning: return "LEFT_RUNNING";
    case RingState::RightRunning: return "RIGHT_RUNNING";
    case RingState::LeftOut: return "LEFT_OUT";
    case RingState::RightOut: return "RIGHT_OUT";
    case RingState::LeftEnd: return "LEFT_END";
    case RingState::RightEnd: return "RIGHT_END";
    }
    return "UNKNOWN";
}

} // namespace bird

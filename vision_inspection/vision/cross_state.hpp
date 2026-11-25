#pragma once

#include <opencv2/opencv.hpp>
#include <cstddef>
//本文件中定义了十字路口状态机的枚举类型和类，包括状态的更新逻辑
namespace bird {

// 十字状态阶段：None 为普通巡线，Begin 为接近/确认，In 为通过路口内部。
enum class CrossState {
    None,   // CROSS_NONE：普通循迹
    Begin,  // CROSS_BEGIN：检测到十字角点
    In      // CROSS_IN：进入十字内部
};

// 按多帧检测、角点距离和边线暂失/恢复推进状态，避免单帧噪声直接切换。
struct CrossStateMachine {
    CrossState state = CrossState::None;

    int seenFrames = 0;
    int lostFrames = 0;
    int recoveredFrames = 0;
    int crossLostFrames = 0;
    bool sawBothLinesLost = false;

    CrossState update(bool crossFound, const cv::Point2f& leftCorner,
        const cv::Point2f& rightCorner, const cv::Point2f& vehiclePoint,
        std::size_t leftCount, std::size_t rightCount)
    {
        constexpr int confirmFrames = 2;
        constexpr float cornerNearPixels = 130.f;
        constexpr std::size_t shortLineCount = 20;
        constexpr int lostConfirmFrames = 2;
        constexpr int recoveredConfirmFrames = 3;
        constexpr int falseCrossRecoveryFrames = 6;

        const float leftDistance = static_cast<float>(cv::norm(leftCorner - vehiclePoint));
        const float rightDistance = static_cast<float>(cv::norm(rightCorner - vehiclePoint));

        const bool cornerNear =
            leftDistance < cornerNearPixels ||
            rightDistance < cornerNearPixels;

        const bool bothLinesShort =
            leftCount < shortLineCount &&
            rightCount < shortLineCount;
        const bool bothLinesLong =
            leftCount >= shortLineCount &&
            rightCount >= shortLineCount;

        switch (state) {
        case CrossState::None:
            crossLostFrames = 0;

            if (crossFound) {
                ++seenFrames;
            } else {
                seenFrames = 0;
            }

            if (seenFrames >= confirmFrames) {
                state = CrossState::Begin;
                crossLostFrames = 0;
            }
            break;

        case CrossState::Begin:
            if (crossFound) {
                crossLostFrames = 0;

                if (cornerNear || bothLinesShort) {
                    state = CrossState::In;
                }
            } else {
                ++crossLostFrames;

                if (crossLostFrames >= 4) {
                    seenFrames = 0;
                    crossLostFrames = 0;
                    state = CrossState::None;
                }
            }
            break;

        case CrossState::In:
            // A false entry may never see the loss of both edges expected at a real cross.
            crossLostFrames = !crossFound && bothLinesLong ? crossLostFrames + 1 : 0;
            if (bothLinesShort) {
                ++lostFrames;
                recoveredFrames = 0;

                if (lostFrames >= lostConfirmFrames) {
                    sawBothLinesLost = true;
                }
            } else {
                lostFrames = 0;

                if (sawBothLinesLost && bothLinesLong) {
                    ++recoveredFrames;
                } else {
                    recoveredFrames = 0;
                }
            }

            if (recoveredFrames >= recoveredConfirmFrames ||
                crossLostFrames >= falseCrossRecoveryFrames) {
                *this = CrossStateMachine{};
            }
            break;
        }

        return state;
    }
};

} // namespace bird

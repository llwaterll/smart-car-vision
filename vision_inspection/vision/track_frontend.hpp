#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace track {

// 阶段 1：前端配置。集中管理白色、蓝色的 HSV 范围和小区域处理参数；
// 默认值仅用于起步调试，需用实际赛道相机画面校准。
// OpenCV 8-bit HSV: H=0..179, S/V=0..255.
struct FrontendConfig {
    int whiteMaxS = 85;
    int whiteMinV = 110;
    int blueMinH = 90;
    int blueMaxH = 135;
    int blueMinS = 70;
    int blueMinV = 30;
    int closeKernel = 3;        // 1 disables closing; use a small odd kernel.
    int maxHoleArea = 1500;    // fill only enclosed non-blue holes; 0 disables.
    double seedTopRatio = 0.55; // do not acquire distant track across a gap.
    double minRunRatio = 0.12;
    int seedMargin = 5;
    int blueProbe = 12;
    double minBlueRatio = 0.5;

    void validate() const {
        if (whiteMaxS < 0 || whiteMaxS > 255 || whiteMinV < 0 || whiteMinV > 255 ||
            blueMinH < 0 || blueMaxH > 179 || blueMinH > blueMaxH ||
            blueMinS < 0 || blueMinS > 255 || blueMinV < 0 || blueMinV > 255 ||
            closeKernel < 1 || closeKernel > 15 || closeKernel % 2 == 0 ||
            maxHoleArea < 0 || maxHoleArea > 10000 ||
            !std::isfinite(seedTopRatio) || seedTopRatio < 0.2 || seedTopRatio > 0.9 ||
            !std::isfinite(minRunRatio) || minRunRatio <= 0 || minRunRatio > 0.8 ||
            seedMargin < 2 || seedMargin > 30 || blueProbe < 1 || blueProbe > 50 ||
            !std::isfinite(minBlueRatio) || minBlueRatio <= 0 || minBlueRatio > 1) {
            throw std::invalid_argument("Invalid track frontend configuration");
        }
    }
};

// 阶段 2：读取外部参数文件，让调阈值不必重新编译程序。
inline FrontendConfig loadConfig(const std::string& path) {
    FrontendConfig c;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("Cannot open config: " + path);
    auto read = [&](const char* name, auto& value) {
        const cv::FileNode node = fs[name];
        if (!node.empty()) node >> value;
    };
    read("white_max_s", c.whiteMaxS); read("white_min_v", c.whiteMinV);
    read("blue_min_h", c.blueMinH); read("blue_max_h", c.blueMaxH);
    read("blue_min_s", c.blueMinS); read("blue_min_v", c.blueMinV);
    read("close_kernel", c.closeKernel); read("max_hole_area", c.maxHoleArea);
    read("seed_top_ratio", c.seedTopRatio); read("min_run_ratio", c.minRunRatio);
    read("seed_margin", c.seedMargin); read("blue_probe", c.blueProbe);
    read("min_blue_ratio", c.minBlueRatio);
    c.validate();
    return c;
}

// 前端单帧输出：掩膜用于诊断，seed 是近端赛道起点；blueSupport 只是
// 两侧采样到蓝色背景的比例，不是分类置信度或成功概率。
struct Observation {
    cv::Mat whiteMask; // raw white candidates
    cv::Mat blueMask;  // explicit blue exclusions
    cv::Mat roadMask;  // processed candidates, NOT a guaranteed drivable map
    bool seedFound = false;
    int seedY = -1;
    int leftX = -1;
    int rightX = -1;
    double blueSupport = 0; // weaker of the two side probes, NOT probability
};

// 阶段 3：修补赛道区域里的小型封闭缺口（如彩色薄片造成的洞）。
// 连通到图像边缘的背景和含蓝色像素的区域都不填，防止把蓝底断口补成赛道。
inline void fillSmallNonBlueHoles(cv::Mat& mask, const cv::Mat& blue, int maxArea) {
    if (maxArea == 0) return;
    cv::Mat inverse, labels, stats, centroids;
    cv::bitwise_not(mask, inverse);
    const int count = cv::connectedComponentsWithStats(inverse, labels, stats, centroids, 8, CV_32S);
    std::vector<bool> allowed(count, false);
    for (int id = 1; id < count; ++id) {
        const int x = stats.at<int>(id, cv::CC_STAT_LEFT);
        const int y = stats.at<int>(id, cv::CC_STAT_TOP);
        const int w = stats.at<int>(id, cv::CC_STAT_WIDTH);
        const int h = stats.at<int>(id, cv::CC_STAT_HEIGHT);
        const int area = stats.at<int>(id, cv::CC_STAT_AREA);
        allowed[id] = area <= maxArea && x > 0 && y > 0 &&
            x + w < mask.cols && y + h < mask.rows;
    }
    // Never fill a component containing blue pixels (including blue gaps).
    for (int y = 0; y < mask.rows; ++y) {
        const int* ids = labels.ptr<int>(y);
        const uchar* b = blue.ptr<uchar>(y);
        for (int x = 0; x < mask.cols; ++x) if (b[x]) allowed[ids[x]] = false;
    }
    for (int y = 0; y < mask.rows; ++y) {
        const int* ids = labels.ptr<int>(y);
        uchar* dst = mask.ptr<uchar>(y);
        for (int x = 0; x < mask.cols; ++x) if (allowed[ids[x]]) dst[x] = 255;
    }
}

// 阶段 4：从近端向前找赛道起点。候选区域须有足够宽度、未被画面边缘
// 截断，且左右邻域都能看到蓝底，减少误把白色物体当赛道的机会。
inline void findSeed(Observation& out, const FrontendConfig& c) {
    const int w = out.roadMask.cols, h = out.roadMask.rows;
    const int margin = std::max(c.seedMargin, c.blueProbe);
    const int minWidth = std::max(3, static_cast<int>(w * c.minRunRatio));
    if (w <= 2 * margin || h <= 2 * c.seedMargin) return;
    const int nearTop = static_cast<int>(h * c.seedTopRatio);
    // A seam on the optical axis may leave an unbroken white strip just beside it.
    // Keep the same column across rows: separate white patches cannot bridge a gap.
    const int offset = std::max(1, c.blueProbe / 2);
    const int probes[] = {w / 2 - offset, w / 2, w / 2 + offset};
    bool connected[] = {true, true, true};
    for (int y = h - 1 - c.seedMargin; y >= std::min(nearTop, h / 3); --y) {
        const uchar* row = out.roadMask.ptr<uchar>(y);
        const uchar* blue = out.blueMask.ptr<uchar>(y);
        for (int i = 0; i < 3; ++i) {
            const int x = probes[i];
            connected[i] = connected[i] && row[x - 1] && row[x] && row[x + 1];
        }
        if (y < nearTop && !connected[0] && !connected[1] && !connected[2]) break;
        double bestScore = -1e9;
        int bestL = -1, bestR = -1;
        double bestSupport = 0;
        for (int x = 0; x < w;) {
            if (!row[x]) { ++x; continue; }
            const int l = x;
            while (x < w && row[x]) ++x;
            const int r = x - 1;
            if (y < nearTop) {
                bool reachesNear = false;
                for (int i = 0; i < 3; ++i)
                    if (connected[i] && l <= probes[i] - 1 && probes[i] + 1 <= r) reachesNear = true;
                if (!reachesNear) continue;
            }
            if (r - l + 1 < minWidth || l < margin || r >= w - margin) continue;
            int lb = 0, rb = 0;
            for (int k = 1; k <= c.blueProbe; ++k) {
                lb += blue[l - k] != 0; rb += blue[r + k] != 0;
            }
            const double support = static_cast<double>(std::min(lb, rb)) / c.blueProbe;
            if (support < c.minBlueRatio) continue;
            // Prefer the near, central road, not simply the widest bright object.
            const double score = (r - l + 1) - 0.5 * std::abs((l + r) * 0.5 - w * 0.5);
            if (score > bestScore) {
                bestScore = score; bestL = l; bestR = r; bestSupport = support;
            }
        }
        if (bestL >= 0) {
            out.seedFound = true; out.seedY = y; out.leftX = bestL; out.rightX = bestR;
            out.blueSupport = bestSupport;
            return;
        }
    }
}

struct SingleEdgeSeed {
    int x = -1, y = -1;
    int side = 0; // +1 left edge, -1 right edge
};

inline SingleEdgeSeed findSingleEdgeSeed(const cv::Mat& roadMask, const cv::Mat& blueMask,
                                         const FrontendConfig& c, int priorY, int priorLeft, int priorRight) {
    SingleEdgeSeed best;
    constexpr int rowTolerance = 36;
    constexpr int columnTolerance = 64;
    int bestScore = 3 * columnTolerance;
    const int margin = std::max(c.seedMargin, c.blueProbe);
    const int minWidth = std::max(3, static_cast<int>(roadMask.cols * c.minRunRatio));
    const int firstRow = std::max(priorY - rowTolerance, static_cast<int>(roadMask.rows * c.seedTopRatio));
    const int lastRow = std::min(priorY + rowTolerance, roadMask.rows - 1 - c.seedMargin);
    for (int y = firstRow; y <= lastRow; ++y) {
        const uchar* road = roadMask.ptr<uchar>(y);
        const uchar* blue = blueMask.ptr<uchar>(y);
        for (int x = 0; x < roadMask.cols;) {
            if (!road[x]) { ++x; continue; }
            const int left = x;
            while (x < roadMask.cols && road[x]) ++x;
            const int right = x - 1;
            if (right - left + 1 < minWidth) continue;
            int leftBlue = 0, rightBlue = 0;
            for (int k = 1; k <= c.blueProbe; ++k) {
                leftBlue += left >= margin && blue[left - k] != 0;
                rightBlue += right < roadMask.cols - margin && blue[right + k] != 0;
            }
            const bool hasLeft = leftBlue >= c.blueProbe * c.minBlueRatio;
            const bool hasRight = rightBlue >= c.blueProbe * c.minBlueRatio;
            if (hasLeft == hasRight) continue;
            const int edge = hasLeft ? left : right;
            const int delta = std::abs(edge - (hasLeft ? priorLeft : priorRight));
            const int score = delta + 2 * std::abs(y - priorY);
            if (delta <= columnTolerance && score < bestScore) {
                best = {edge, y, hasLeft ? 1 : -1};
                bestScore = score;
            }
        }
    }
    return best;
}

// 阶段 5：把前端步骤串成单帧处理：BGR 转 HSV -> 白色候选/蓝底掩膜
// -> 形态学小修补 -> 强制排除蓝底 -> 起点搜索。返回结果供边线跟踪使用。
inline Observation segment(const cv::Mat& bgr, const FrontendConfig& c) {
    c.validate();
    Observation out;
    if (bgr.empty()) return out;
    if (bgr.type() != CV_8UC3) throw std::invalid_argument("Frontend requires CV_8UC3 BGR input");
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, c.whiteMinV), cv::Scalar(179, c.whiteMaxS, 255), out.whiteMask);
    cv::inRange(hsv, cv::Scalar(c.blueMinH, c.blueMinS, c.blueMinV),
                cv::Scalar(c.blueMaxH, 255, 255), out.blueMask);
    out.roadMask = out.whiteMask.clone();
    if (c.closeKernel > 1) {
        cv::morphologyEx(out.roadMask, out.roadMask, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(c.closeKernel, c.closeKernel)));
    }
    out.roadMask.setTo(0, out.blueMask);
    fillSmallNonBlueHoles(out.roadMask, out.blueMask, c.maxHoleArea);
    findSeed(out, c);
    return out;
}

} // namespace track

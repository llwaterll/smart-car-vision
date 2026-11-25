#include "img_handle.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
    std::cout << "PASS: " << message << '\n';
}
cv::Mat road() {
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(180, 50, 20));
    cv::rectangle(image, cv::Rect(200, 0, 240, 480), cv::Scalar(220, 220, 220), cv::FILLED);
    return image;
}

void checkCrossRecovery() {
    using bird::CrossState;
    const cv::Point2f vehicle(300.f, 460.f), left(245.f, 380.f), right(355.f, 380.f);
    bird::CrossStateMachine cross;
    const auto update = [&](bool found, std::size_t leftCount, std::size_t rightCount) {
        return cross.update(found, left, right, vehicle, leftCount, rightCount);
    };
    const auto enter = [&] {
        update(true, 30, 30);
        check(update(true, 30, 30) == CrossState::Begin &&
              update(true, 30, 30) == CrossState::In, "confirmed near corners enter the cross");
    };
    const auto goodFrames = [&](int count) {
        for (int i = 0; i < count; ++i) update(false, 30, 30);
        return cross.state;
    };

    enter();
    check(goodFrames(5) == CrossState::In, "brief two-edge recovery does not leave the cross");
    update(true, 30, 30);
    check(goodFrames(5) == CrossState::In, "new cross corners reset the fallback count");
    update(false, 19, 30);
    check(goodFrames(5) == CrossState::In, "a short edge resets the fallback count");
    check(goodFrames(1) == CrossState::None, "six clean two-edge frames clear a false cross");

    enter();
    update(false, 10, 10);
    check(update(false, 10, 10) == CrossState::In, "a real cross continues while both edges are short");
    check(goodFrames(2) == CrossState::In, "a real cross waits for three recovered frames");
    check(goodFrames(1) == CrossState::None, "a real cross keeps the original recovery path");
}

void checkRingRecovery() {
    using bird::RingState;
    std::vector<cv::Point2f> left, right, curvedRight;
    for (int i = 0; i < 60; ++i) {
        const float row = 460.f - 5.f * i;
        left.emplace_back(205.f, row);
        right.emplace_back(395.f, row);
        curvedRight.emplace_back(395.f - 0.14f * i * i, row);
    }

    bird::RingStateMachine ring;
    ring.state = RingState::LeftIn;
    for (int i = 0; i < 5; ++i)
        check(ring.update(false, false, false, false, left, right) == RingState::LeftIn,
              "an early ring state survives a brief straight segment");
    check(ring.update(false, false, false, false, left, right) == RingState::None,
          "two sustained straight boundaries clear a false LEFT_IN");

    ring.state = RingState::RightBegin;
    for (int i = 0; i < 6; ++i)
        ring.update(false, false, false, false, left, right);
    check(ring.state == RingState::None, "the same recovery clears a false RIGHT_BEGIN");

    auto diagonalLeft = left;
    auto diagonalRight = right;
    for (std::size_t i = 0; i < left.size(); ++i) {
        const float offset = 2.f * static_cast<float>(i);
        diagonalLeft[i].x -= offset;
        diagonalRight[i].x -= offset;
    }
    ring.state = RingState::LeftIn;
    for (int i = 0; i < 5; ++i)
        check(ring.update(false, false, false, false, diagonalLeft, diagonalRight) == RingState::LeftIn,
              "parallel diagonal edges cannot advance LEFT_IN to LEFT_RUNNING");
    check(ring.update(false, false, false, false, diagonalLeft, diagonalRight) == RingState::None,
          "the diagonal straight corridor also clears a false LEFT_IN");

    ring.state = RingState::LeftIn;
    for (int i = 0; i < 3; ++i)
        ring.update(false, false, false, false, left, curvedRight);
    check(ring.state == RingState::LeftRunning, "turn evidence still advances a genuine left entry");
}

void checkRingEntryRejectsBriefNotch() {
    std::vector<cv::Point2f> left, right;
    for (int i = 0; i < 65; ++i) {
        left.emplace_back(205.f, 460.f - 5.f * i);
        right.emplace_back(395.f, 460.f - 5.f * i);
    }
    auto notched = left;
    notched[8].x += 11.f;
    cv::Point2f corner;
    float angle = 0.f;
    check(bird::findCorner(notched, corner, angle) && bird::hasStraightCorridor(notched, right),
          "short stripe-like notch produces a corner within an otherwise straight corridor");
    check(!bird::detectLeftRingBegin(notched, right, corner), "straight corridor vetoes a false left ring entry");
    bird::RingStateMachine ring;
    ring.state = bird::RingState::LeftIn;
    for (int i = 0; i < 6; ++i)
        ring.update(false, false, true, false, notched, right);
    check(ring.state == bird::RingState::None, "a local notch cannot prevent recovery to ordinary tracking");
    notched = right;
    notched[8].x -= 11.f;
    check(!bird::detectRightRingBegin(left, notched, corner), "straight corridor vetoes a false right ring entry");

    auto mouth = left;
    for (int i = 15; i < 65; ++i) mouth[i] = {205.f - 5.f * (i - 15), 385.f};
    check(bird::detectLeftRingBegin(mouth, right, corner), "a sustained opening still qualifies for a left ring");
}

void checkStraightPreview() {
    const cv::Point2f vehicle(300.f, 460.f);
    const cv::Point2f near(287.f, 380.f); // ~3 cm, as in the captured packet
    std::vector<cv::Point2f> left, right, bendLeft, bendRight;
    for (int i = 0; i < 60; ++i) {
        const float row = 460.f - 5.f * i;
        const float bend = 0.08f * i * i;
        left.emplace_back(191.f, row);
        right.emplace_back(383.f, row);
        bendLeft.emplace_back(191.f + bend, row);
        bendRight.emplace_back(383.f + bend, row);
    }
    const auto far = bird::straightRoadPreview(left, right, vehicle, near, 192.f);
    const auto old = control::calculatePurePursuit(vehicle, near, config::kMetersPerPixel, config::kWheelBaseMeters);
    const auto corrected = control::calculatePurePursuit(vehicle, far, config::kMetersPerPixel, config::kWheelBaseMeters);
    check(far.y < near.y && far.x == near.x && corrected.valid &&
          corrected.curvature < 0.f && std::abs(corrected.curvature) < std::abs(old.curvature) * 0.4f,
          "straight road extends lookahead and reduces overcorrection without losing steering direction");
    const cv::Point2f nearBend(287.f + 0.08f * 16.f * 16.f, 380.f);
    check(bird::straightRoadPreview(bendLeft, bendRight, vehicle, nearBend, 192.f) == nearBend,
          "a bent corridor keeps the original short lookahead");
    right.resize(23);
    check(bird::straightRoadPreview(left, right, vehicle, near, 192.f) == near,
          "missing far boundary never selects an extrapolated target");
}

void checkSingleEdgeRecovery() {
    const cv::Scalar blueColor(180,50,20), white(220,220,220);
    cv::Mat measured(480, 640, CV_8UC3, blueColor);
    cv::Mat leftOnly = measured.clone(), rightOnly = measured.clone();
    for (int y = 0; y < measured.rows; ++y) {
        const int bend = (479 - y) * (479 - y) / 12000;
        const int left = 238 - y * 197 / 479 + bend;
        const int right = 418 + y * 157 / 479 + bend;
        cv::line(measured, {left,y}, {right,y}, white);
        cv::line(leftOnly, {left,y}, {639,y}, white);
        cv::line(rightOnly, {0,y}, {right,y}, white);
    }
    ProcessOptions options;
    options.mode = TrackMode::Hsv;
    const auto frame = [&](const cv::Mat& image, double ms) {
        options.sourceTimeMs = ms;
        return img_handle(image, false, options);
    };
    const auto estimated = [](const FrameResult& result) {
        return (result.wireObservation.validMask & vision_link::CurvatureEstimated) != 0;
    };

    resetVisionState();
    check(!frame(leftOnly, 0).previewFound, "a single edge without recent measured geometry is ignored");
    auto full = frame(measured, 40);
    check(full.previewFound && !estimated(full), "two visible edges seed measured road geometry");
    for (int i = 1; i <= 4; ++i) {
        auto partial = frame(leftOnly, 40 + 40*i);
        check(partial.previewFound && estimated(partial) && partial.wireObservation.confidence == 450 &&
              std::abs(partial.pursuit.lateralError - full.pursuit.lateralError) < 0.025f,
              "clipped right edge briefly reuses the measured width and center");
    }
    check(!frame(leftOnly, 240).previewFound, "persistent single edge exhausts the four-frame limit");
    full = frame(measured, 280);
    check(full.previewFound && estimated(frame(rightOnly, 320)), "a new full measurement enables right-edge recovery");
    check(!frame(rightOnly, 531).previewFound, "single-edge recovery expires after 250 ms");

    frame(measured, 600);
    cv::Mat gap(480, 640, CV_8UC3, blueColor);
    auto missing = frame(gap, 640);
    check(!missing.previewFound && !estimated(missing) && missing.wireObservation.element == vision_link::Element::Gap,
          "a blue gap retains its existing element and does not invent steering");
    check(!frame(leftOnly, 680).previewFound, "no stale single-edge recovery after an intervening gap");
}

int main(int argc, char** argv) {
    try {
        checkCrossRecovery();
        checkRingRecovery();
        checkRingEntryRejectsBriefNotch();
        checkStraightPreview();
        checkSingleEdgeRecovery();
        const track::FrontendConfig c;
        auto straight = road();
        auto a = track::segment(straight, c);
        check(a.seedFound && a.leftX == 200 && a.rightX == 439, "white road on blue background has expected seed");
        auto dim = straight.clone(); dim.convertTo(dim, -1, 0.65);
        check(track::segment(dim, c).seedFound, "moderately dim synthetic road still acquired");
        check(!track::segment(cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255)), c).seedFound,
              "all-white frame rejected without blue support");
        cv::Mat blue(480, 640, CV_8UC3, cv::Scalar(180,50,20));
        check(!track::segment(blue, c).seedFound, "all-blue frame rejected");
        auto grayBackground = road();
        grayBackground(cv::Rect(0,0,200,480)).setTo(cv::Scalar(20,20,20));
        grayBackground(cv::Rect(440,0,200,480)).setTo(cv::Scalar(20,20,20));
        check(!track::segment(grayBackground, c).seedFound, "HSV mode rejects dark non-blue background");
        for (const auto& color : {cv::Scalar(0,0,220), cv::Scalar(0,220,0), cv::Scalar(0,220,220)}) {
            auto obstacle = road(); cv::circle(obstacle, cv::Point(320,455), 14, color, cv::FILLED);
            auto o = track::segment(obstacle, c);
            check(o.seedFound && o.roadMask.at<uchar>(455,320) == 255,
                  "small enclosed colored hole does not split the road seed");
        }
        auto blueHole = road(); cv::circle(blueHole, {320,400}, 14, cv::Scalar(180,50,20), cv::FILLED);
        check(track::segment(blueHole, c).roadMask.at<uchar>(400,320) == 0, "blue hole never filled");
        auto gap = road();
        cv::rectangle(gap, cv::Rect(0,260,640,220), cv::Scalar(180,50,20), cv::FILLED);
        check(!track::segment(gap, c).seedFound, "distant white road beyond gap is not acquired as near road");
        auto narrowGap = road(); narrowGap.row(350).setTo(cv::Scalar(180,50,20));
        check(track::segment(narrowGap, c).roadMask.at<uchar>(350,320) == 0, "closing does not bridge blue strip");
        auto edgeRoad = road(); cv::rectangle(edgeRoad, cv::Rect(0,0,210,480), cv::Scalar(220,220,220), cv::FILLED);
        check(!track::segment(edgeRoad, c).seedFound, "road with clipped boundary is reported unavailable");
        auto clippedNear = road();
        clippedNear(cv::Rect(0, 200, 220, 280)).setTo(cv::Scalar(220,220,220));
        check(track::segment(clippedNear, c).seedFound, "continuous road can acquire both edges above a clipped near section");
        cv::Mat seamedNear = clippedNear.clone();
        cv::rectangle(seamedNear, cv::Rect(318, 300, 5, 180), cv::Scalar(35,35,35), cv::FILLED);
        check(track::segment(seamedNear, c).seedFound, "narrow center seam leaves a continuous adjacent road path");
        seamedNear.row(230).setTo(cv::Scalar(180,50,20));
        check(!track::segment(seamedNear, c).seedFound, "adjacent continuity cannot bridge an actual gap");
        clippedNear.row(230).setTo(cv::Scalar(180,50,20));
        check(!track::segment(clippedNear, c).seedFound, "extended seed search cannot cross a blue strip");
        check(!track::segment(cv::Mat(), c).seedFound, "empty input has no seed");
        bool rejected = false;
        try { auto invalid = c; invalid.closeKernel = 4; invalid.validate(); } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "even morphology kernel rejected");
        auto contour = mazeWalk(a.roadMask, a.leftX + 1, a.seedY, 1, true);
        check(!contour.empty(), "binary boundary walker finds road boundary");
        bool inside = true;
        for (const auto& p : contour) inside = inside && a.roadMask.at<uchar>(p) != 0;
        check(inside, "binary boundary walker never steps onto blue background");
        check(mazeWalk(cv::Mat::zeros(480,640,CV_8UC1),320,450,1,true).empty(), "black mask does not become traversable");
        ProcessOptions options; options.mode = TrackMode::Hsv;
        auto offTrack = grayBackground.clone();
        cv::circle(offTrack, {320,350}, 16, cv::Scalar(0,0,220), cv::FILLED);
        resetVisionState();
        FrameResult clutter;
        for (int i = 0; i < 8; ++i) clutter = img_handle(offTrack, false, options);
        check(clutter.obstacles.hasPrimary && clutter.wireObservation.element == vision_link::Element::None,
              "off-track color remains diagnostic without publishing a race element");
        resetVisionState();
        (void)img_handle(straight, false, options);
        const auto lost = img_handle(blue, false, options);
        check(!lost.seedFound && !lost.previewFound && !lost.pursuit.valid && lost.leftPoints == 0 && lost.rightPoints == 0,
              "valid-looking previous frame does not leak output into a lost-track frame");
        bool badSize = false;
        try { (void)img_handle(cv::Mat(240,320,CV_8UC3), false, options); } catch (const cv::Exception&) { badSize = true; }
        check(badSize, "wrong calibration resolution rejected");
        if (argc == 2) {
            const std::filesystem::path dir(argv[1]);
            if (!std::filesystem::create_directory(dir)) throw std::runtime_error("Fixture directory already exists");
            cv::imwrite((dir / "straight.png").string(), straight);
            cv::imwrite((dir / "gap.png").string(), gap);
            cv::VideoWriter writer((dir / "sequence.avi").string(), cv::VideoWriter::fourcc('M','J','P','G'),10,{640,480});
            if (!writer.isOpened()) throw std::runtime_error("Cannot create test video");
            for (int i=0; i<5; ++i) writer.write(straight);
            for (int i=0; i<5; ++i) writer.write(blue);
            for (int i=0; i<5; ++i) writer.write(straight);
        }
        std::cout << "All frontend tests passed. Synthetic images do not validate driving.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}

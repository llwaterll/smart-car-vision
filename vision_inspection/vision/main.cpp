#include <opencv2/opencv.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include "img_handle.hpp"
#include "gap_state.hpp"
#include "obstacle_task.hpp"
#include "vision_serial.hpp"
#include "sudoku_reader.hpp"

namespace fs = std::filesystem;

// 命令行参数：选择图像/视频/摄像头输入、前端模式、日志目录和显示方式。
struct Arguments {
    std::string source, path, output, configPath, visionUart;
    TrackMode mode = TrackMode::Hsv;
    bool tcp = false, show = false, sendHello = false, http = false;
    int httpPort = HTTP_PORT;
    int maxFrames = 0, saveEvery = 1;
};

void usage() {
    std::cout << "Usage: ./vision (--image FILE | --video FILE | --camera INDEX) [options]\n"
        "  --mode hsv|gray       default hsv; gray uses the assessment pipeline\n"
        "  --config FILE         HSV thresholds (otherwise built-in defaults)\n"
        "  --output NEW_DIR      CSV and diagnostic PNGs; directory must not exist\n"
        "  --save-every N        save every Nth frame, default 1 (CSV keeps all)\n"
        "  --max-frames N        stop after N frames, 0 means unlimited\n"
        "  --show               local windows; ESC/q quits, SPACE pauses replay\n"
        "  --tcp                enable original TCP viewer on port 9000\n"
        "  --http               enable browser viewer; open http://<board-ip>:8080/\n"
        "  --http-port N        browser viewer port, default 8080\n"
        "  --vision-uart DEV    live camera: send track and Sudoku tiles on UART (115200)\n"
        "  --hello              repeat this session's HELLO every second; start while F407 is STOPPED\n"
        "No actuator commands are transmitted. Inputs must be 640x480.\n";
}

int integer(const std::string& text, const char* option, int minimum) {
    std::size_t used = 0;
    int n = std::stoi(text, &used);
    if (used != text.size() || n < minimum) throw std::invalid_argument(std::string("Invalid ") + option);
    return n;
}

// 启动阶段：解析并检查命令行参数，避免输入源、阈值文件或输出路径无效时
// 程序运行到一半才失败。
Arguments parse(int argc, char** argv) {
    Arguments a;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("Missing value: " + key);
            return argv[i];
        };
        if (key == "--image" || key == "--video" || key == "--camera") {
            if (!a.source.empty()) throw std::invalid_argument("Choose exactly one input source");
            a.source = key; a.path = value();
        } else if (key == "--mode") {
            const auto v = value();
            if (v != "hsv" && v != "gray") throw std::invalid_argument("Mode must be hsv or gray");
            a.mode = v == "hsv" ? TrackMode::Hsv : TrackMode::Gray;
        } else if (key == "--config") a.configPath = value();
        else if (key == "--output") a.output = value();
        else if (key == "--max-frames") a.maxFrames = integer(value(), "--max-frames", 0);
        else if (key == "--save-every") a.saveEvery = integer(value(), "--save-every", 1);
        else if (key == "--tcp") a.tcp = true;
        else if (key == "--http") a.http = true;
        else if (key == "--http-port") a.httpPort = integer(value(), "--http-port", 1);
        else if (key == "--show") a.show = true;
        else if (key == "--vision-uart") a.visionUart = value();
        else if (key == "--hello") a.sendHello = true;
        else throw std::invalid_argument("Unknown argument: " + key);
    }
    if (a.source.empty()) throw std::invalid_argument("An input source is required; use --help");
    if (a.httpPort > 65535) throw std::invalid_argument("Invalid --http-port");
    if (!a.visionUart.empty() && a.source != "--camera")
        throw std::invalid_argument("Vision UART output is allowed only with live --camera input, never image/video replay");
    return a;
}

// 运行总流程：准备输入与配置 -> 逐帧调用视觉处理 -> 输出统计/CSV/调试图。
// 本程序只分析图像，不向电机或底盘发送控制命令。
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") { usage(); return 0; }
        const auto args = parse(argc, argv);
        ProcessOptions options;
        options.mode = args.mode;
        if (!args.configPath.empty()) options.frontend = track::loadConfig(args.configPath);
        if (args.mode == TrackMode::Hsv) {
            std::cerr << "HSV thresholds are provisional; validate them on venue recordings.\n";
        }
        // 阶段 1：打开唯一的数据源。图片用于单帧检查，视频用于可重复回放，
        // 摄像头用于实时观察；三种入口共用后面的同一处理链路。
        cv::VideoCapture capture;
        cv::Mat image;
        const bool liveCamera = args.source == "--camera";
        const int cameraIndex = liveCamera ? integer(args.path, "--camera", 0) : 0;
        if (args.source == "--image") {
            image = cv::imread(args.path, cv::IMREAD_COLOR);
            if (image.empty()) throw std::runtime_error("Cannot read image: " + args.path);
        } else if (args.source == "--video") {
            if (!capture.open(args.path)) throw std::runtime_error("Cannot open video: " + args.path);
        }
        // 阶段 2：按需启用远程看图，并准备本次运行的输出目录和逐帧 CSV。
        // 输出目录必须是新目录，避免误覆盖已有测试结果。
        if (args.sendHello && args.visionUart.empty())
            throw std::invalid_argument("--hello requires --vision-uart DEV");
        std::unique_ptr<vision_link::SerialSender> visionSender;
        std::uint32_t visionSession = 0;
        if (!args.visionUart.empty()) {
            std::random_device random;
            visionSession = (static_cast<std::uint32_t>(random()) << 1) ^ static_cast<std::uint32_t>(random());
            if (visionSession == 0) visionSession = 1;
            visionSender = std::make_unique<vision_link::SerialSender>(args.visionUart, visionSession);
            std::cerr << "Vision UART enabled at 115200 8N1; verify F407 is STOPPED before --hello.\n";
            visionSender->startCameraStatus();
        }
        if (args.tcp) remote.enable();
        if (args.http) remote.enableHttp(static_cast<uint16_t>(args.httpPort));
        std::ofstream csv, sudokuCsv;
        if (!args.output.empty()) {
            if (!fs::create_directory(args.output)) throw std::runtime_error("Output directory already exists");
            csv.open(fs::path(args.output) / "frames.csv");
            if (!csv) throw std::runtime_error("Cannot create frames.csv");
            sudokuCsv.open(fs::path(args.output) / "sudoku.csv");
            if (!sudokuCsv) throw std::runtime_error("Cannot create sudoku.csv");
            sudokuCsv << "frame,tile,c0,c1,c2,c3,c4,c5,c6,c7,c8,age_ms,sent\n";
            csv << "frame,source_ms,processing_ms,mode,seed_found,seed_y,blue_support,left_points,right_points,cross,ring,side,preview_found,pp_valid,lateral_m,curvature,steering_deg,gap_phase,gap_fault,gap_distance_m,gap_heading_hold,gap_hold_last_steering,gap_speed_mps,obstacle_count,obstacle_color,obstacle_x,obstacle_y,obstacle_area_px,obstacle_confidence,obstacle_stable,obstacle_event_id,obstacle_phase,obstacle_action,obstacle_event_color,obstacle_event_x,obstacle_event_y,obstacle_event_confidence,obstacle_command_valid,obstacle_fault\n";
        }
        resetVisionState();
        sudoku::Reader tileReader;
        int frames = 0, valid = 0;
        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        double processTotalMs = 0;
        auto nextHello = Clock::time_point::min();
        bool helloSent = false;
        bool cameraWarning = false;
        // 阶段 3：逐帧处理。记录来源时间和处理耗时；CSV 对每帧留档，
        // 图片按 saveEvery 抽样，减少长视频产生的磁盘占用。
        while (args.maxFrames == 0 || frames < args.maxFrames) {
            cv::Mat current;
            double sourceMs = 0;
            if (args.source == "--image") {
                if (frames > 0) break;
                current = image;
            } else if (liveCamera) {
                const char* cameraError = nullptr;
                if (!capture.isOpened()) {
                    if (capture.open(cameraIndex, cv::CAP_V4L2)) {
                        capture.set(cv::CAP_PROP_FRAME_WIDTH, config::kInputWidth);
                        capture.set(cv::CAP_PROP_FRAME_HEIGHT, config::kInputHeight);
                        capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
                    } else {
                        cameraError = "Cannot open camera";
                    }
                }
                if (!cameraError && (!capture.read(current) || current.empty()))
                    cameraError = "Input read failed";
                if (cameraError) {
                    if (!visionSender) throw std::runtime_error(cameraError);
                    visionSender->setCameraStreaming(false);
                    if (!cameraWarning) std::cerr << cameraError << "; retrying camera.\n";
                    cameraWarning = true;
                    capture.release();
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
                cameraWarning = false;
                if (visionSender) visionSender->setCameraStreaming(true);
                sourceMs = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            } else {
                if (!capture.read(current) || current.empty()) {
                    if (frames == 0) throw std::runtime_error("Input read failed");
                    break; // end of a decoded video
                }
                sourceMs = capture.get(cv::CAP_PROP_POS_MSEC);
            }
            const auto frameReceived = Clock::now();
            if (visionSender && args.sendHello && frameReceived >= nextHello && visionSender->sendHello()) {
                helloSent = true;
                nextHello = frameReceived + std::chrono::seconds(1);
            }
            const bool canSendBusiness = visionSender && (!args.sendHello || helloSent);
            const auto t0 = frameReceived;
            options.sourceTimeMs = sourceMs;
            const auto result = img_handle(current, false, options);
            const auto frameReady = Clock::now();
            const double ms = std::chrono::duration<double, std::milli>(frameReady - t0).count();
            processTotalMs += ms;
            valid += result.pursuit.valid;
            if (canSendBusiness) {
                // 这是当前进程可测的采集后年龄；不包含相机曝光前和 F407 接收队列延迟。
                if (ms < 200) visionSender->sendTrack(static_cast<uint16_t>(std::ceil(ms)), result.wireObservation);
            }
            if (frames % 2 == 0) {
                for (const auto& tile:tileReader.readAll(current)) {
                    const auto age = std::chrono::duration<double,std::milli>(Clock::now()-frameReceived).count();
                    const bool sent = canSendBusiness && age <= 200 &&
                        visionSender->sendTile(static_cast<uint16_t>(std::ceil(age)),tile.index,tile.cells);
                    if (sudokuCsv.is_open()) {
                        sudokuCsv << frames << ',' << unsigned(tile.index);
                        for (auto cell:tile.cells) sudokuCsv << ',' << unsigned(cell);
                        sudokuCsv << ',' << age << ',' << sent << '\n';
                        if (!sudokuCsv) throw std::runtime_error("Sudoku CSV write failed");
                    }
                }
            }
            remote.imshow("Camera", current);
            // 阶段 4：记录结构化逐帧指标并按指定间隔保存输入和中间结果，
            // 这样既能定位某帧误检，也能控制长录像占用空间。
            if (csv.is_open()) {
                csv << frames << ',' << sourceMs << ',' << ms << ','
                    << (args.mode == TrackMode::Hsv ? "hsv" : "gray") << ','
                    << result.seedFound << ',' << result.seedY << ',' << result.blueSupport << ','
                    << result.leftPoints << ',' << result.rightPoints << ','
                    << static_cast<int>(result.cross) << ',' << bird::ringStateName(result.ring) << ','
                    << result.selectedSide << ',' << result.previewFound << ',' << result.pursuit.valid << ','
                    << result.pursuit.lateralError << ',' << result.pursuit.curvature << ','
                    << result.pursuit.steeringDegrees << ',' << task::gapPhaseName(result.gap.phase) << ','
                    << task::gapFaultName(result.gap.fault) << ',' << result.gap.distanceFromEntryM << ','
                    << result.gap.useHeadingHold << ',' << result.gap.holdLastSteering << ','
                    << result.gap.suggestedSpeedMps << ',' << result.obstacles.candidates.size();
                if (result.obstacles.hasPrimary) {
                    const auto& d = result.obstacles.primary;
                    csv << ',' << obstacle::colorName(d.color) << ',' << d.center.x << ',' << d.center.y
                        << ',' << d.areaPx << ',' << d.confidence << ',' << d.stable;
                } else {
                    csv << ",NONE,0,0,0,0,0";
                }
                const auto& event = result.obstacleEvent;
                csv << ',' << event.eventId << ',' << obstacle_task::phaseName(event.phase) << ','
                    << obstacle_task::actionName(event.action) << ',' << obstacle::colorName(event.color) << ','
                    << event.centerPx.x << ',' << event.centerPx.y << ',' << event.confidence << ','
                    << event.commandValid << ',' << event.fault;
                csv << '\n';
                if (!csv) throw std::runtime_error("CSV write failed");
                if (frames % args.saveEvery == 0) {
                    const std::string prefix = cv::format("%06d_", frames);
                    auto save = [&](const char* suffix, const cv::Mat& picture) {
                        if (!picture.empty() && !cv::imwrite((fs::path(args.output) / (prefix + suffix + ".png")).string(), picture))
                            throw std::runtime_error("Image write failed");
                    };
                    save("input", current); save("mask", result.mask); save("blue", result.blueMask);
                    save("edges", result.overlay); save("bird", result.birdImage); save("obstacles", result.obstacles.overlay);
                }
            }
            ++frames;
            // 阶段 4：可选的交互预览。离线视频可暂停逐帧检查，ESC/q 结束。
            if (args.show) {
                cv::imshow("Camera", current); cv::imshow("edges", result.overlay); cv::imshow("bird", result.birdImage);
                if (!result.mask.empty()) cv::imshow("mask", result.mask);
                if (!result.blueMask.empty()) cv::imshow("blue", result.blueMask);
                int delay = args.source == "--image" ? 0 : 1;
                if (args.source == "--video") {
                    const double fps = capture.get(cv::CAP_PROP_FPS);
                    if (std::isfinite(fps) && fps > 0) delay = std::clamp(static_cast<int>(1000.0 / fps - ms), 1, 1000);
                }
                int key = cv::waitKey(delay) & 0xff;
                if (key == ' ' && args.source == "--video") key = cv::waitKey(0) & 0xff;
                if (key == 27 || key == 'q') break;
            }
        }
        // 收尾阶段：显式刷新日志并汇总帧数、有效预瞄帧数和平均处理耗时。
        if (csv.is_open()) { csv.flush(); if (!csv) throw std::runtime_error("CSV flush failed"); }
        if (sudokuCsv.is_open()) { sudokuCsv.flush(); if (!sudokuCsv) throw std::runtime_error("Sudoku CSV flush failed"); }
        std::cout << "frames=" << frames << " pp_valid_frames=" << valid
                  << " mean_processing_ms=" << (frames ? processTotalMs / frames : 0)
                  << " (not a real-vehicle success rate)\n";
        return frames ? 0 : 1;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }
}

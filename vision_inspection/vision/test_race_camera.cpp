#include "img_handle.hpp"
#include "sudoku_reader.hpp"
#include "vision_serial.hpp"
#include <poll.h>
#include <pty.h>
extern "C" {
#include "Car_Race.h"
}
#include <cassert>
#include <iostream>

static void checkSerialBackpressure() {
    int master, slave;
    char path[128];
    assert(openpty(&master, &slave, path, nullptr, nullptr) == 0);
    vision_link::SerialSender uart(path, 123);
    assert(uart.sendHello());

    unsigned sentTracks = 0;
    while (sentTracks < 4096 && uart.sendTrack(10, {})) ++sentTracks;
    assert(sentTracks > 0 && sentTracks < 4096);

    CarVision rx;
    CarVision_Init(&rx);
    const auto receiveUntil = [&](unsigned sequence) {
        while (rx.last_sequence != sequence) {
            pollfd ready{master, POLLIN, 0};
            assert(::poll(&ready, 1, 1000) == 1);
            uint8_t bytes[256];
            const auto n = ::read(master, bytes, sizeof(bytes));
            assert(n > 0);
            for (ssize_t i = 0; i < n; ++i) CarVision_Feed(&rx, bytes[i], 40, 1);
        }
    };
    receiveUntil(sentTracks);

    // Model a truncated write in front of the next complete frame.
    const auto partial = vision_link::encodeTrack(sentTracks + 1, 123, 10, {});
    assert(::write(slave, partial.data(), 10) == 10);
    assert(uart.sendTrack(10, {}));
    receiveUntil(sentTracks + 1);
    CarVisionObservation resumed{};
    assert(CarVision_GetObservation(&rx, 40, &resumed) && resumed.sequence == sentTracks + 1);

    ::close(master);
    bool fatal = false;
    try { uart.sendHello(); }
    catch (const std::runtime_error&) { fatal = true; }
    assert(fatal);
    ::close(slave);
}

int main(int argc, char** argv) {
    checkSerialBackpressure();
    resetVisionState();
    ProcessOptions options;
    options.mode=TrackMode::Hsv;
    auto track=img_handle(cv::imread("test_fixtures/straight.png"),false,options);
    assert(track.pursuit.valid && (track.wireObservation.validMask&vision_link::CurvatureValid));
    assert(track.wireObservation.curvatureMilli==std::lround(-1000*track.pursuit.curvature));
    // Run the Linux UART sender on a pseudoterminal and decode its actual bytes
    // with the firmware receiver. This checks both coordinate signs and the link.
    int master,slave;
    char path[128];
    assert(openpty(&master,&slave,path,nullptr,nullptr)==0);
    vision_link::SerialSender uart(path,123);
    CarVision rx;
    CarVision_Init(&rx);
    auto receive=[&](unsigned count) {
        while (count) {
            uint8_t bytes[48];
            auto n=::read(master,bytes,std::min<size_t>(count,sizeof(bytes)));
            assert(n>0);
            for (int i=0;i<n;++i) CarVision_Feed(&rx,bytes[i],40,1);
            count-=n;
        }
    };
    uart.sendHello();
    uart.sendTrack(10,track.wireObservation);
    receive(16+32);
    CarVisionObservation obs{};
    assert(CarVision_GetObservation(&rx,40,&obs));
    CarRace race;
    Race_Start(&race);
    auto wheels=Race_Step(&race,&obs,0,0,0.005f,0,0,1,40);
    assert((wheels.rpm[1]-wheels.rpm[0])*track.pursuit.curvature<0);
    assert(!CarVision_GetObservation(&rx,231,&obs));
    auto gap=img_handle(cv::imread("test_fixtures/gap.png"),false,options);
    assert(!gap.wireObservation.validMask && gap.wireObservation.element==vision_link::Element::Gap);

    sudoku::Reader reader;
    auto dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    const std::array<sudoku::Cells,4> puzzle={{
        {1,0,3,0,5,0,2,0,1}, {4,0,0,0,2,3,5,0,4},
        {5,6,0,0,1,2,6,0,0}, {0,3,1,6,0,5,0,1,0}
    }};
    uint8_t tiles[4][9], solution[36];
    std::vector<cv::Mat> unrotated;
    cv::VideoWriter recording;
    if (argc == 2) {
        recording.open(argv[1],cv::VideoWriter::fourcc('M','J','P','G'),25,{640,480});
        assert(recording.isOpened());
    }
    for (int tile=0;tile<4;++tile) {
        const auto& digits=puzzle[tile];
        cv::Mat frame(400,500,CV_8UC1,cv::Scalar(255));
        for (int t=0;t<2;++t) {
            cv::Mat mark;
            cv::aruco::drawMarker(dictionary,tile*2+t,40,mark);
            mark.copyTo(frame(cv::Rect(t?320:94,t?290:64,40,40)));
        }
        for (int k=0;k<=3;++k) {
            cv::line(frame,{140+58*k,110},{140+58*k,284},0,2);
            cv::line(frame,{140,110+58*k},{314,110+58*k},0,2);
        }
        for (int i=0;i<9;++i) if (digits[i])
            cv::putText(frame,std::to_string(digits[i]),{156+(i%3)*58,151+(i/3)*58},
                        cv::FONT_HERSHEY_SIMPLEX,1.1,0,2);
        cv::Mat bgr;
        cv::cvtColor(frame,bgr,cv::COLOR_GRAY2BGR);
        unrotated.push_back(bgr.clone());
        if (recording.isOpened()) {
            cv::Mat camera(480,640,CV_8UC3,cv::Scalar(255,255,255));
            bgr.copyTo(camera(cv::Rect(70,40,bgr.cols,bgr.rows)));
            for (unsigned repeat=0;repeat<6;++repeat) recording.write(camera);
        }
        if (tile%2) cv::rotate(bgr,bgr,cv::ROTATE_90_CLOCKWISE);
        assert(!reader.read(bgr));
        auto result=reader.read(bgr);
        assert(result && result->index==tile && result->cells==digits);
        uart.sendTile(15,result->index,result->cells);
        receive(26);
        CarVisionTile received{};
        assert(CarVision_GetTile(&rx,tile,40,&received));
        assert(std::equal(digits.begin(),digits.end(),received.cells));
        std::copy_n(received.cells,9,tiles[tile]);
    }
    assert(Race_Solve(tiles,solution));
    const std::array<uint8_t,36> expected={1,2,3,4,5,6,4,5,6,1,2,3,2,3,1,5,6,4,
        5,6,4,2,3,1,3,1,2,6,4,5,6,4,5,3,1,2};
    assert(std::equal(expected.begin(),expected.end(),solution));
    cv::Mat both(400,1000,CV_8UC3,cv::Scalar(255,255,255));
    unrotated[0].copyTo(both(cv::Rect(0,0,500,400)));
    unrotated[1].copyTo(both(cv::Rect(500,0,500,400)));
    sudoku::Reader multi;
    assert(multi.readAll(both).empty());
    auto batch=multi.readAll(both);
    assert(batch.size()==2 && batch[0].index==0 && batch[1].index==1);
    for (const auto& tile:batch) {
        assert(tile.cells==puzzle[tile.index]);
        uart.sendTile(15,tile.index,tile.cells);
    }
    receive(52);
    assert(rx.tiles[0].cells[0]==puzzle[0][0] && rx.tiles[1].cells[0]==puzzle[1][0]);
    assert(multi.readAll(cv::Mat(400,1000,CV_8UC3,cv::Scalar(255,255,255))).empty());
    assert(multi.readAll(both).empty()); // a missed observation resets confirmation
    ::close(master); ::close(slave);
    std::cout<<"PASS UART backpressure and recovery; camera to UART to wheels; four AprilTag/OCR tiles to F407 puzzle solution; multi-tile frames\n";
}

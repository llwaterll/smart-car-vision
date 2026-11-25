#include "vision_link.hpp"
#include <iostream>
#include <stdexcept>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
    std::cout << "PASS: " << message << '\n';
}

int main() {
    try {
        const char* test = "123456789";
        check(vision_link::crc16(reinterpret_cast<const std::uint8_t*>(test), 9) == 0x29b1,
              "CRC-16/CCITT-FALSE standard check vector matches 0x29B1");
        vision_link::TrackObservation o;
        const std::vector<std::uint8_t> expected = {
            0xa5,0x5a,0x01,0x01,0x00,0x10,0x00,0x01,0x00,0x00,0x00,0x7b,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0x00,
            0x00,0x00,0xca,0xa0
        };
        const auto frame = vision_link::encodeTrack(1, 123, 0, o);
        check(frame == expected && frame.size() == vision_link::kTrackFrameSize,
              "empty TYPE=1 frame matches repository Python reference vector");
        const auto hello = vision_link::encodeHello(123);
        const std::vector<std::uint8_t> expectedHello = {
            0xa5,0x5a,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7b,0x00,0x00,0xaf,0x75
        };
        check(hello == expectedHello, "HELLO frame matches expected v1 layout and CRC");
        const auto status = vision_link::encodeCameraStatus(123, false);
        const auto statusCrc = vision_link::crc16(status.data() + 2, status.size() - 4);
        check(status.size() == 17 && status[3] == 3 && status[5] == 1 && status[11] == 123 &&
              status[14] == 0 && status[15] == statusCrc >> 8 && status[16] == (statusCrc & 0xff) &&
              vision_link::encodeCameraStatus(123, true)[14] == 1,
              "camera status reports both conditions independently of business sequence");
        o.element = vision_link::Element::RedObstacle;
        o.confidence = 875;
        const auto red = vision_link::encodeTrack(7, 0x12345678, 30, o);
        check(red.size() == 32 && red[2] == 1 && red[3] == 1 && red[24] == 6 && red[25] == 255,
              "red obstacle maps to element 6 with unknown tag id");
        bool rejected = false;
        try { auto invalid = o; invalid.reserved = 1; (void)vision_link::encodeTrack(1, 1, 0, invalid); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "nonzero reserved field rejected");
        rejected = false;
        try { (void)vision_link::encodeTrack(1, 1, 201, o); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "processing age above 200 ms rejected");
        rejected = false;
        try { (void)vision_link::encodeHello(0); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "zero session rejected");
        check(vision_link::obstacleElement(1) == vision_link::Element::GreenObstacle &&
              vision_link::obstacleElement(2) == vision_link::Element::YellowObstacle &&
              vision_link::obstacleElement(3) == vision_link::Element::RedObstacle,
              "detector color ids map to F407 elements 4/5/6");
        std::cout << "All vision link tests passed. UART hardware remains untested.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}

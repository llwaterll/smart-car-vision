#pragma once

#include "vision_link.hpp"
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <fcntl.h>

namespace vision_link {

// Linux/Orange Pi 串口发送器。仅发送香橙派 -> F407 的观测帧，不接收 ACK。
class SerialSender {
public:
    SerialSender(const std::string& device, std::uint32_t session)
        : session_(session) {
        if (device.empty() || session == 0) throw std::invalid_argument("Serial device and nonzero session are required");
        fd_ = ::open(device.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("Cannot open vision UART: " + std::string(std::strerror(errno)));
        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) { close(); throw std::runtime_error("tcgetattr failed"); }
        cfmakeraw(&tty);
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
#ifdef CRTSCTS
        tty.c_cflag &= ~CRTSCTS;
#endif
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); throw std::runtime_error("tcsetattr failed"); }
        tcflush(fd_, TCIOFLUSH);
    }

    SerialSender(const SerialSender&) = delete;
    SerialSender& operator=(const SerialSender&) = delete;
    ~SerialSender() {
        statusRunning_ = false;
        if (statusThread_.joinable()) statusThread_.join();
        close();
    }

    // Keep reporting even if opening or reading the camera stalls.
    void startCameraStatus() {
        statusRunning_ = true;
        statusThread_ = std::thread([this] {
            while (statusRunning_) {
                const auto last = cameraFrameMs_.load();
                try {
                    writeFrame(encodeCameraStatus(session_, last != 0 && nowMs() - last <= 1000));
                } catch (const std::runtime_error&) {
                    // A blocked UART naturally expires the receiver's status.
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }
        });
    }

    void setCameraStreaming(bool streaming) { cameraFrameMs_ = streaming ? nowMs() : 0; }

    // Announce the unchanged session. The F407 binds a new session only when stopped.
    bool sendHello() { return writeFrame(encodeHello(session_)); }

    // Send current track/element observation and advance the shared business sequence.
    bool sendTrack(std::uint16_t processingAgeMs, const TrackObservation& observation) {
        if (!writeFrame(encodeTrack(sequence_, session_, processingAgeMs, observation))) return false;
        sequence_ = static_cast<std::uint16_t>(sequence_ + 1);
        return true;
    }

    bool sendTile(uint16_t age, uint8_t index, const std::array<uint8_t,9>& cells) {
        if (!writeFrame(encodeTile(sequence_,session_,age,index,cells))) return false;
        ++sequence_;
        return true;
    }
private:
    static std::int64_t nowMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool writeFrame(const std::vector<std::uint8_t>& bytes) {
        const std::lock_guard<std::mutex> lock(writeMutex_);
        std::size_t offset = 0;
        while (offset < bytes.size()) {
            const ssize_t n = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
            if (n > 0) { offset += static_cast<std::size_t>(n); continue; }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                // Drop any unsent suffix; the F407 parser resynchronizes on the next frame.
                return false;
            }
            if (n == 0) throw std::runtime_error("Vision UART write made no progress");
            throw std::runtime_error("Vision UART write failed: " + std::string(std::strerror(errno)));
        }
        return true;
    }

    int fd_ = -1;
    std::uint32_t session_;
    std::uint16_t sequence_ = 1; // HELLO uses seq=0; business frames start at 1.
    std::mutex writeMutex_;
    std::atomic<std::int64_t> cameraFrameMs_{0};
    std::atomic<bool> statusRunning_{false};
    std::thread statusThread_;
};

} // namespace vision_link

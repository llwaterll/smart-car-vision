#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace vision_link {

constexpr std::uint8_t kVersion = 1;
constexpr std::uint8_t kTypeHello = 0;
constexpr std::uint8_t kTypeTrack = 1;
constexpr std::uint8_t kTypeSudokuTile = 2;
constexpr std::uint8_t kTypeCameraStatus = 3;
constexpr std::size_t kTrackPayloadSize = 16;
constexpr std::size_t kTrackFrameSize = 32; // 2 sync + 12 header + 16 payload + 2 CRC

enum class Element : std::uint8_t {
    None = 0, Ring = 1, Cross = 2, Gap = 3,
    GreenObstacle = 4, YellowObstacle = 5, RedObstacle = 6,
    FinishCandidate = 7, Tag = 8
};

enum ValidMask : std::uint16_t {
    LateralValid = 1u << 0,
    HeadingValid = 1u << 1,
    CurvatureValid = 1u << 2,
    DistanceValid = 1u << 3,
    CurvatureEstimated = 1u << 4
};

// Fields use the units/ranges defined by the control repository's TYPE=1 contract.
struct TrackObservation {
    std::uint16_t validMask = 0;
    std::uint16_t confidence = 0;       // 0..1000
    std::int16_t lateralMm = 0;         // -1000..1000, left positive
    std::int16_t headingMrad = 0;       // -3142..3142, CCW positive
    std::int16_t curvatureMilli = 0;    // -20000..20000, 0.001 / m
    Element element = Element::None;
    std::uint8_t tagId = 255;
    std::uint16_t distanceMm = 0;       // 0..65535; only meaningful with DistanceValid
    std::uint16_t reserved = 0;         // Protocol requires zero for v1.
};

inline void appendU16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value >> 8));
    out.push_back(static_cast<std::uint8_t>(value & 0xff));
}
inline void appendI16(std::vector<std::uint8_t>& out, std::int16_t value) {
    appendU16(out, static_cast<std::uint16_t>(value));
}
inline void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value >> 24));
    out.push_back(static_cast<std::uint8_t>(value >> 16));
    out.push_back(static_cast<std::uint8_t>(value >> 8));
    out.push_back(static_cast<std::uint8_t>(value));
}

// CRC-16/CCITT-FALSE, matches the F407 receiver and Python reference encoder.
inline std::uint16_t crc16(const std::uint8_t* data, std::size_t size) {
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<std::uint16_t>(crc << 1);
    }
    return crc;
}

inline void validate(const TrackObservation& o) {
    if (o.validMask > 31 || ((o.validMask & CurvatureEstimated) && !(o.validMask & CurvatureValid)) ||
        o.confidence > 1000 ||
        o.lateralMm < -1000 || o.lateralMm > 1000 ||
        o.headingMrad < -3142 || o.headingMrad > 3142 ||
        o.curvatureMilli < -20000 || o.curvatureMilli > 20000 ||
        static_cast<std::uint8_t>(o.element) > 8 ||
        (o.tagId > 7 && o.tagId != 255) || o.reserved != 0)
        throw std::invalid_argument("TYPE=1 observation outside protocol range");
}

// Encode a TYPE=1 frame. The sequence belongs to all business messages in one session.
inline std::vector<std::uint8_t> encodeTrack(std::uint16_t sequence, std::uint32_t session,
                                             std::uint16_t processingAgeMs,
                                             const TrackObservation& o) {
    validate(o);
    if (session == 0 || processingAgeMs > 200)
        throw std::invalid_argument("Invalid vision session or processing age");
    std::vector<std::uint8_t> out;
    out.reserve(kTrackFrameSize);
    out.push_back(0xa5); out.push_back(0x5a);
    out.push_back(kVersion); out.push_back(kTypeTrack);
    appendU16(out, static_cast<std::uint16_t>(kTrackPayloadSize));
    appendU16(out, sequence);
    appendU32(out, session);
    appendU16(out, processingAgeMs);
    appendU16(out, o.validMask);
    appendU16(out, o.confidence);
    appendI16(out, o.lateralMm);
    appendI16(out, o.headingMrad);
    appendI16(out, o.curvatureMilli);
    out.push_back(static_cast<std::uint8_t>(o.element));
    out.push_back(o.tagId);
    appendU16(out, o.distanceMm);
    appendU16(out, o.reserved);
    appendU16(out, crc16(out.data() + 2, out.size() - 2));
    return out;
}

// HELLO declares a new nonzero sender session. Only call while the F407 car is STOPPED.
inline std::vector<std::uint8_t> encodeHello(std::uint32_t session) {
    if (session == 0) throw std::invalid_argument("Vision session must be nonzero");
    std::vector<std::uint8_t> out;
    out.reserve(16);
    out.push_back(0xa5); out.push_back(0x5a);
    out.push_back(kVersion); out.push_back(kTypeHello);
    appendU16(out, 0);
    appendU16(out, 0); // HELLO sequence is defined as zero.
    appendU32(out, session);
    appendU16(out, 0); // HELLO age is defined as zero.
    appendU16(out, crc16(out.data() + 2, out.size() - 2));
    return out;
}

// Diagnostic only. Zero sequence and age keep it separate from race observations.
inline std::vector<std::uint8_t> encodeCameraStatus(std::uint32_t session, bool streaming) {
    if (session == 0) throw std::invalid_argument("Vision session must be nonzero");
    std::vector<std::uint8_t> out{0xa5,0x5a,kVersion,kTypeCameraStatus};
    appendU16(out, 1);
    appendU16(out, 0);
    appendU32(out, session);
    appendU16(out, 0);
    out.push_back(streaming ? 1 : 0);
    appendU16(out, crc16(out.data() + 2, out.size() - 2));
    return out;
}

inline std::vector<uint8_t> encodeTile(uint16_t sequence, uint32_t session, uint16_t age,
                                       uint8_t index, const std::array<uint8_t,9>& cells) {
    if (!session || age > 200 || index > 3 ||
        std::any_of(cells.begin(),cells.end(),[](uint8_t x){return x>6;}))
        throw std::invalid_argument("Invalid sudoku observation");
    std::vector<uint8_t> out{0xa5,0x5a,kVersion,kTypeSudokuTile};
    appendU16(out,10); appendU16(out,sequence); appendU32(out,session); appendU16(out,age);
    out.push_back(index); out.insert(out.end(),cells.begin(),cells.end());
    appendU16(out,crc16(out.data()+2,out.size()-2));
    return out;
}

inline Element obstacleElement(std::uint8_t color) {
    switch (color) {
    case 1: return Element::GreenObstacle;
    case 2: return Element::YellowObstacle;
    case 3: return Element::RedObstacle;
    default: return Element::None;
    }
}

inline std::uint16_t confidencePermille(double confidence) {
    if (!std::isfinite(confidence)) return 0;
    return static_cast<std::uint16_t>(std::clamp(std::lround(confidence * 1000.0), 0l, 1000l));
}

} // namespace vision_link

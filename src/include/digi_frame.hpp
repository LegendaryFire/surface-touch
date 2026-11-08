// SPDX-License-Identifier: MIT
// Container contract for the Surface digitizer object container 0x1A.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace digi {

// Fixed 4350-byte containers, id 0x1A (declared == wire).
inline constexpr std::size_t kWireSize = 4350;
inline constexpr std::uint8_t kReportId = 0x1A;

// Header anchors.
inline constexpr std::size_t kOffTs16 = 1;          // LE, 0.1 ms (the pad's unit)
inline constexpr std::size_t kOffClass = 3;
inline constexpr std::size_t kOffSecLen = 10;
inline constexpr std::size_t kOffMatrixLen = 22;
inline constexpr std::size_t kOffCells = 26;
inline constexpr std::uint16_t kClassMatrix = 0x1082;  // all 2550 matrix frames
inline constexpr std::uint16_t kClassScene = 0x10F2;   // all 1698 scene frames
inline constexpr std::uint32_t kSecLenMatrix = 4072;
inline constexpr std::uint32_t kSecLenScene = 4331;
inline constexpr std::uint32_t kMatrixCells = 4056;    // 78 x 52

// Geometry from the device's 0x06 DeviceParameters (rows@15=52, cols@19=78):
// 3.891 mm square cells over 303.48 x 202.32 mm.
inline constexpr unsigned kCols = 78;
inline constexpr unsigned kRows = 52;

// Tail (268 B) = the group-4 record-list section that follows the matrix.
// kOffStateA/B are entries 2 and 15 of its type-0x04 record's 16-entry array
// (a firmware value). The bytes are kept in full; nothing reads them for
// meaning yet.
inline constexpr std::size_t kOffTail = kOffCells + kMatrixCells;  // 4082
inline constexpr std::size_t kWireTailEnd = kWireSize;             // 4350
inline constexpr std::size_t kOffStateA = 0x102D;   // type-0x04 entry 2
inline constexpr std::size_t kOffStateB = 0x1061;   // type-0x04 entry 15

// Cell semantics: idle 0xB4 (the same gate value as the pad), touch darkens.
// The decode threshold is the band in digi::DeviceConfig, derived from this
// device's 0x06 block 0.
inline constexpr std::uint8_t kCellIdle = 0xB4;

enum class Fail { None, ShortRead, ReportId, Class, SecLen, MatrixLen };

struct Frame {
  std::uint16_t ts16;          // 0.1 ms
  bool matrix;                 // class: true = matrix, false = scene
  std::uint32_t sec_len;       // validated to the class constant
  const std::uint8_t* cells;   // matrix: 4056 cells @26; scene: nullptr
  const std::uint8_t* tail;    // 268 B @4082 (both classes)
  std::uint8_t state_a;        // raw (entry 2 low byte)
  std::uint8_t state_b;        // raw (entry 15 low byte)
};

inline std::uint16_t le16(const std::uint8_t* p) {
  return std::uint16_t(p[0]) | std::uint16_t(p[1]) << 8;
}
inline std::uint32_t le32(const std::uint8_t* p) {
  return std::uint32_t(p[0]) | std::uint32_t(p[1]) << 8 |
         std::uint32_t(p[2]) << 16 | std::uint32_t(p[3]) << 24;
}

// Every anchor must match for the class its class tag declares; a mismatch
// rejects the whole frame.
inline std::optional<Frame> parse_container(const std::uint8_t* buf,
                                            std::size_t n,
                                            Fail* why = nullptr) {
  auto bad = [&](Fail f) -> std::optional<Frame> {
    if (why) *why = f;
    return std::nullopt;
  };
  if (n < kWireSize) return bad(Fail::ShortRead);
  if (buf[0] != kReportId) return bad(Fail::ReportId);
  const bool matrix = le16(buf + kOffClass) == kClassMatrix;
  if (!matrix && le16(buf + kOffClass) != kClassScene) return bad(Fail::Class);
  const std::uint32_t sec = le32(buf + kOffSecLen);
  if (sec != (matrix ? kSecLenMatrix : kSecLenScene)) return bad(Fail::SecLen);
  const std::uint32_t mat = le32(buf + kOffMatrixLen);
  if (mat != (matrix ? kMatrixCells : 0u)) return bad(Fail::MatrixLen);
  if (why) *why = Fail::None;
  return Frame{le16(buf + kOffTs16), matrix, sec,
               matrix ? buf + kOffCells : nullptr, buf + kOffTail,
               buf[kOffStateA], buf[kOffStateB]};
}

}  // namespace digi

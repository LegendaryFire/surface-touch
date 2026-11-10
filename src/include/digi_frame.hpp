// SPDX-License-Identifier: MIT
// The digitizer's touch frames: 4350-byte reports, id 0x1A. A frame is
// either a cell matrix or a scene frame without one.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "wire.hpp"

namespace digi {

inline constexpr std::size_t kWireSize = 4350;
inline constexpr std::uint8_t kReportId = 0x1A;

inline constexpr std::size_t kOffTs16 = 1;  // 0.1 ms units
inline constexpr std::size_t kOffClass = 3;
inline constexpr std::size_t kOffSecLen = 10;
inline constexpr std::size_t kOffMatrixLen = 22;
inline constexpr std::size_t kOffCells = 26;
inline constexpr std::uint16_t kClassMatrix = 0x1082;
inline constexpr std::uint16_t kClassScene = 0x10F2;
inline constexpr std::uint32_t kSecLenMatrix = 4072;
inline constexpr std::uint32_t kSecLenScene = 4331;
inline constexpr std::uint32_t kMatrixCells = 4056;  // 78 x 52

// 3.891 mm square cells over 303.48 x 202.32 mm.
inline constexpr unsigned kCols = 78;
inline constexpr unsigned kRows = 52;

// After the matrix: 268 bytes of firmware state, not decoded.
inline constexpr std::size_t kOffTail = kOffCells + kMatrixCells;
inline constexpr std::size_t kWireTailEnd = kWireSize;
inline constexpr std::size_t kOffStateA = 0x102D;
inline constexpr std::size_t kOffStateB = 0x1061;

// Idle cells read 0xB4; a finger pulls them down.
inline constexpr std::uint8_t kCellIdle = 0xB4;

enum class Fail { None, ShortRead, ReportId, Class, SecLen, MatrixLen };

struct Frame {
  std::uint16_t ts16;
  bool matrix;
  std::uint32_t sec_len;
  const std::uint8_t* cells;  // nullptr in a scene frame
  const std::uint8_t* tail;
  std::uint8_t state_a;
  std::uint8_t state_b;
};

using wire::le16;
using wire::le32;

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

// SPDX-License-Identifier: MIT
// The pad's heat frames: 1000-byte reports, id 0x0A, about 125 Hz.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "wire.hpp"

namespace heat {

inline constexpr std::size_t kFrameSize = 1000;
inline constexpr std::uint8_t kReportId = 0x0A;
inline constexpr std::size_t kOffTs16 = 1;  // 0.1 ms units
inline constexpr std::size_t kOffMagic0 = 3;
inline constexpr std::size_t kOffMagic1 = 4;
inline constexpr std::size_t kOffSecLen = 10;
inline constexpr std::uint32_t kSecLen = 0x1E4;
inline constexpr std::size_t kOffMatrixLen = 22;
inline constexpr std::uint32_t kMatrixCells = 468;  // 26 x 18
inline constexpr std::size_t kOffMatrix = 26;
inline constexpr std::size_t kTailOffset = 494;
// The pad's click. No report carries it; this byte reads 0x00 when nothing
// touches the pad, 0x80 while touched and 0x86 while pressed.
inline constexpr std::size_t kOffPhysState = 635;
inline constexpr std::uint8_t kPhysStateTracked = 0x80;
inline constexpr std::uint8_t kPhysStateButton = 0x06;
// Frames arrive 78..82 ticks apart; a larger step means frames were lost.
inline constexpr std::uint16_t kTsGapLow = 40;
inline constexpr std::uint16_t kTsGapHigh = 120;

enum class Fail { None, ShortRead, ReportId, Magic, SecLen, MatrixLen };

struct Frame {
  std::uint16_t ts16;
  const std::uint8_t* cells;
  std::uint32_t sec_len;
  std::uint32_t matrix_len;
  std::uint8_t phys = 0;
};

using wire::le32;

inline std::optional<Frame> parse_frame(const std::uint8_t* buf, std::size_t n,
                                        Fail* why = nullptr) {
  auto bad = [&](Fail f) -> std::optional<Frame> {
    if (why) *why = f;
    return std::nullopt;
  };
  if (n < kFrameSize) return bad(Fail::ShortRead);
  if (buf[0] != kReportId) return bad(Fail::ReportId);
  if (buf[kOffMagic0] != 0x02 || buf[kOffMagic1] != 0x03) return bad(Fail::Magic);
  if (le32(buf + kOffSecLen) != kSecLen) return bad(Fail::SecLen);
  if (le32(buf + kOffMatrixLen) != kMatrixCells) return bad(Fail::MatrixLen);
  if (why) *why = Fail::None;
  return Frame{static_cast<std::uint16_t>(le32(buf + kOffTs16) & 0xFFFF),
               buf + kOffMatrix, le32(buf + kOffSecLen),
               le32(buf + kOffMatrixLen), buf[kOffPhysState]};
}

// How many frames were lost between two consecutive ones.
inline unsigned ts_gap_frames(std::uint16_t prev, std::uint16_t cur) {
  int d = static_cast<std::uint16_t>(cur - prev);
  if (d <= kTsGapHigh) return 0;
  unsigned missed = 0;
  while (d > kTsGapHigh) {
    d = static_cast<std::uint16_t>(d - kTsGapHigh);
    ++missed;
  }
  return missed;
}

}  // namespace heat

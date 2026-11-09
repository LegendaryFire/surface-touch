// SPDX-License-Identifier: MIT
// Frame definitions for the Surface vendor container stream.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "wire.hpp"

namespace heat {

// Fixed 1000-byte reports, id 0x0A, ~125 Hz.
inline constexpr std::size_t kFrameSize = 1000;
inline constexpr std::uint8_t kReportId = 0x0A;
inline constexpr std::size_t kOffTs16 = 1;              // LE, 0.1 ms units
inline constexpr std::size_t kOffMagic0 = 3;            // constant 0x02
inline constexpr std::size_t kOffMagic1 = 4;            // constant 0x03
inline constexpr std::size_t kOffSecLen = 10;           // u32 0x1E4 (484)
inline constexpr std::uint32_t kSecLen = 0x1E4;
inline constexpr std::size_t kOffMatrixLen = 22;        // u32 468 = 26*18
inline constexpr std::uint32_t kMatrixCells = 468;      // 26 x 18 (matches the
                                                        // 0x06 geometry)
inline constexpr std::size_t kOffMatrix = 26;           // raw u8 cells
inline constexpr std::size_t kTailOffset = 494;         // metadata tail, not an
                                                        // input
// Physical device state - the pad's own press state, reported as the PTP
// Button bit. No HID report carries a click: the wire carries a single
// report class (0x0A/1000 B), and the controller hands the kernel nothing
// else.
//
// Exactly one byte of the tail moves with a press - b635: 0x00 surface
// empty / 0x80 tracked, released / 0x86 tracked and pressed.
inline constexpr std::size_t kOffPhysState = 635;
inline constexpr std::uint8_t kPhysStateTracked = 0x80;  // surface in use
inline constexpr std::uint8_t kPhysStateButton = 0x06;   // pressed (both
                                                         // bits together)
// Cadence: ts16 deltas are 78..82 (0.1 ms ticks). A delta outside
// [40, 120] cannot be one frame interval -> counted as a gap.
inline constexpr std::uint16_t kTsGapLow = 40;
inline constexpr std::uint16_t kTsGapHigh = 120;

enum class Fail { None, ShortRead, ReportId, Magic, SecLen, MatrixLen };

struct Frame {
  std::uint16_t ts16;                        // 0.1 ms units
  const std::uint8_t* cells;                 // 468 bytes into the buffer
  std::uint32_t sec_len;
  std::uint32_t matrix_len;
  std::uint8_t phys = 0;                 // b635 physical device state (above)
};

using wire::le32;

// A frame is valid only if every anchor matches; never partially decoded.
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

// ts16 gap/loss detection between consecutive valid frames. Returns the
// number of frames the gap implies were missed (0 = consecutive).
inline unsigned ts_gap_frames(std::uint16_t prev, std::uint16_t cur) {
  // ts16 wraps at 65536 (= 6.5536 s at 125 Hz); unwrap with the shortest
  // forward distance.
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

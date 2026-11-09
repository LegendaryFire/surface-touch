// SPDX-License-Identifier: MIT
// v1 DeviceParameters decoder — parse the 0x06 feature report and drive
// DeviceConfig from the device's own calibration.
//
// The byte layout was read live from the device. Block 0's min/max/avg
// triple is the touch gate. Checks that come from this engine rather than
// from the format (matrix fit, u8 clamp) are marked at their sites.
//
// Layout of the 120-byte response (bare CapImg container, no v3 magic):
//   [0]      report id 0x06
//   [1..5)   u32 total payload length   (119 = len - 1)
//   [5..8)   zero
//   [8..12)  u32 section length         (112 = len - 8)
//   [12..14) u16 section type           (0x0200 = DeviceParameters)
//   [14]     u8 section version         (1)
//   [15..17) u16 rows                   (SensorRowCount > 0)
//   [19..23) u32 cols                   (SensorColumnCount > 0)
//   [23..27) u32 himetric width  (0.01 mm) (HimetricWidth > 0)
//   [27..31) u32 himetric height (0.01 mm) (HimetricHeight > 0)
//   [31]     u8 flag (possibly an inversion flag; carried, not used)
//   [32..36) f32 scalar (480.0 on this device; meaning unknown)
//   [48..52) f32 scalar (470.588... on this device; meaning unknown)
//   [56 + 16*i] four stat blocks {min, max, avg, x}, i = 0..3:
//       block 0 AC/DC baseline A  \ names: ACBaseline, DCBaseline,
//       block 1 AC/DC baseline B   > TouchSignal, HoverSignal (which of
//       block 2 Touch/Hover pair   > blocks 2/3 is touch and which hover
//       block 3 Touch/Hover pair  /  is not certain; blocks 0/1 are equal)
//   [36..48), [52..56) zeros; response ends at 56 + 64 = 120.
//   Each block satisfies min <= x <= max; the engine treats avg as the x and
//   refuses a block that breaks it.
#pragma once

#include <cstdint>
#include <string>

#include "frame.hpp"        // kMatrixCells
#include "heat_engine.hpp"  // DeviceConfig
#include "wire.hpp"

namespace heat {

struct StatBlock {
  float mn = 0, mx = 0, avg = 0, extra = 0;
};

struct DeviceParams {
  std::uint32_t total = 0;       // container echo, for the artifact line
  std::uint32_t section_len = 0; // container echo
  int rows = 0;
  int cols = 0;
  std::uint32_t him_w = 0;
  std::uint32_t him_h = 0;
  std::uint8_t flag = 0;   // +31; carried, not consumed
  float scalar32 = 0;       // +32; carried, meaning unknown
  float scalar48 = 0;       // +48; carried, meaning unknown
  StatBlock block[4]{};

  // The gate from block 0's min/max/avg, truncated toward zero and clamped
  // to 0..255 (the engine's cells are u8).
  DeviceConfig to_config() const;
};

// Decodes the container and the v1 section and validates the fields. On
// false, err names the first violated rule.
inline bool decode_device_params(const std::uint8_t* b, std::size_t n,
                                 DeviceParams* out, std::string* err) {
  auto fail = [&](const char* m) {
    if (err) *err = m;
    return false;
  };
  auto u16 = [&](std::size_t o) { return wire::le16(b + o); };
  auto u32 = [&](std::size_t o) { return wire::le32(b + o); };
  auto fl = [&](std::size_t o) { return wire::le_f32(b + o); };

  if (n < 120) return fail("buffer shorter than the 120-B v1 response");
  if (b[0] != 0x06) return fail("report id is not 0x06");
  DeviceParams p;
  p.total = u32(1);
  p.section_len = u32(8);
  if (p.total != n - 1) return fail("container total != buffer length - 1");
  if (p.section_len != n - 8) return fail("section length != buffer length - 8");
  if (u16(12) != 0x0200) return fail("section type is not 0x0200 DeviceParameters");
  if (b[14] != 1) return fail("section version is not the decoded v1");

  p.rows = u16(15);
  p.cols = int(u32(19));
  p.him_w = u32(23);
  p.him_h = u32(27);
  if (p.rows <= 0) return fail("SensorRowCount must be > 0");
  if (p.cols <= 0) return fail("SensorColumnCount must be > 0");
  if (p.him_w == 0) return fail("HimetricWidth must be > 0");
  if (p.him_h == 0) return fail("HimetricHeight must be > 0");
  // Engine-side rule: this daemon decodes the 468-cell stream with stride
  // 26; a matrix that cannot map onto the frame parser is refused whole
  // rather than half-applied.
  if (p.rows * p.cols != int(kMatrixCells) || p.cols != 26)
    return fail("matrix does not map onto the 26x18 heat stream (engine-side rule)");

  p.flag = b[31];
  p.scalar32 = fl(32);
  p.scalar48 = fl(48);
  for (int i = 0; i < 4; ++i) {
    p.block[i] = StatBlock{fl(56 + 16 * i), fl(60 + 16 * i), fl(64 + 16 * i),
                           fl(68 + 16 * i)};
    // min <= avg <= max.
    if (!(p.block[i].mn <= p.block[i].avg && p.block[i].avg <= p.block[i].mx))
      return fail("statistics block breaks the min <= avg <= max clamp chain");
  }
  *out = p;
  return true;
}

inline DeviceConfig DeviceParams::to_config() const {
  DeviceConfig c;
  c.rows = rows;
  c.cols = cols;
  c.him_w = him_w;
  c.him_h = him_h;
  auto trunc_u8 = [](float f) -> std::uint8_t {
    int v = static_cast<int>(f);  // truncate toward zero
    if (v < 0) v = 0;             // u8 cell domain
    if (v > 255) v = 255;
    return std::uint8_t(v);
  };
  const StatBlock& b0 = block[0];  // the AC-baseline triple
  // Entry edge: trunc(avg + 4*(min - avg) + 0.5) over the AC-baseline
  // triple — on this machine trunc(180 - 8 + 0.5) = 172, the touch-band
  // ceiling, which separates hover from touch.
  // Equivalently: require a deviation of 4*(avg-min) counts below the
  // baseline, which excludes the hover band.
  c.gate_lo = trunc_u8(b0.avg + 4.0f * (b0.mn - b0.avg) + 0.5f);
  c.gate_hi = trunc_u8(b0.mx);
  c.gate = trunc_u8(b0.avg);  // release edge = trunc(avg)
  c.baseline = trunc_u8(b0.avg);
  // Spawn strength = the device TouchSignal average (block 2): a blob must
  // contain a cell at real touch strength to become a contact.
  c.spawn_gate = trunc_u8(block[2].avg);
  return c;
}

}  // namespace heat

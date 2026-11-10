// SPDX-License-Identifier: MIT
// Decodes the pad's calibration, feature report 0x06, into a DeviceConfig.
//
// The 120-byte response:
//   [0]          report id 0x06
//   [1..5)       u32 length - 1
//   [8..12)      u32 length - 8
//   [12..14)     u16 section type 0x0200
//   [14]         u8 section version 1
//   [15..17)     u16 rows
//   [19..23)     u32 cols
//   [23..27)     u32 width, 0.01 mm
//   [27..31)     u32 height, 0.01 mm
//   [31]         u8 flag, unused
//   [32..36)     f32, unknown (480.0)
//   [48..52)     f32, unknown (470.588)
//   [56 + 16*i]  four f32 blocks {min, max, avg, x}: two baselines, then
//                touch and hover signal
#pragma once

#include <cstdint>
#include <string>

#include "frame.hpp"
#include "heat_engine.hpp"
#include "wire.hpp"

namespace heat {

struct StatBlock {
  float mn = 0, mx = 0, avg = 0, extra = 0;
};

struct DeviceParams {
  std::uint32_t total = 0;
  std::uint32_t section_len = 0;
  int rows = 0;
  int cols = 0;
  std::uint32_t him_w = 0;
  std::uint32_t him_h = 0;
  std::uint8_t flag = 0;
  float scalar32 = 0;
  float scalar48 = 0;
  StatBlock block[4]{};

  DeviceConfig to_config() const;
};

// On false, err says what was wrong.
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
  // The frame parser only knows the 26 x 18 layout.
  if (p.rows * p.cols != int(kMatrixCells) || p.cols != 26)
    return fail("matrix does not map onto the 26x18 heat stream (engine-side rule)");

  p.flag = b[31];
  p.scalar32 = fl(32);
  p.scalar48 = fl(48);
  for (int i = 0; i < 4; ++i) {
    p.block[i] = StatBlock{fl(56 + 16 * i), fl(60 + 16 * i), fl(64 + 16 * i),
                           fl(68 + 16 * i)};
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
    int v = static_cast<int>(f);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return std::uint8_t(v);
  };
  const StatBlock& b0 = block[0];
  // A touch must reach 4x the baseline's spread below its average
  // (172 on this pad); anything shallower is hover.
  c.gate_lo = trunc_u8(b0.avg + 4.0f * (b0.mn - b0.avg) + 0.5f);
  c.gate_hi = trunc_u8(b0.mx);
  c.gate = trunc_u8(b0.avg);
  c.baseline = trunc_u8(b0.avg);
  c.spawn_gate = trunc_u8(block[2].avg);
  return c;
}

}  // namespace heat

// SPDX-License-Identifier: MIT
// Turns decoded pen frames into pen reports. In range and tip down are
// thresholds on the profiles' mean signal; position comes from
// pen_position.hpp, pressure from pen_pressure.hpp.
#pragma once

#include <cstdint>

#include "pen_frame.hpp"
#include "pen_position.hpp"
#include "pen_pressure.hpp"
#include "pen_uhid.hpp"

namespace pen {
namespace feed {

// Measured: hover peaks at 996 and a touching tip never drops below 2047.
inline constexpr std::uint64_t kPresencePm = 50;
inline constexpr std::uint64_t kTipPm = 1521;

// Fallback when the profiles give no position: the controller's own cell.
inline std::uint16_t map_x(std::uint8_t col) {
  const std::int64_t v = (std::int64_t(col) * 4 - 3) *
                         std::int64_t(digi::penhid::kExtentX) / 301;
  if (v <= 0) return 0;
  return v >= std::int64_t(digi::penhid::kExtentX) ? digi::penhid::kExtentX
                                                   : std::uint16_t(v);
}
inline std::uint16_t map_y(std::uint8_t row) {
  const std::int64_t v = (std::int64_t(row) * 4 - 4) *
                         std::int64_t(digi::penhid::kExtentY) / 197;
  if (v <= 0) return 0;
  return v >= std::int64_t(digi::penhid::kExtentY) ? digi::penhid::kExtentY
                                                   : std::uint16_t(v);
}

// The outer cells' centres land on the ends of the range.
inline constexpr double kScaleX = double(digi::penhid::kExtentX) / double(kDimCols - 1);
inline constexpr double kScaleY = double(digi::penhid::kExtentY) / double(kDimRows - 1);
inline std::uint16_t project(double cell, double scale, std::uint16_t extent) {
  const double v = cell * scale + 0.5;
  if (v <= 0.0) return 0;
  return v >= double(extent) ? extent : std::uint16_t(v);
}
inline std::uint16_t map_x_c(double cell) {
  return project(cell, kScaleX, digi::penhid::kExtentX);
}
inline std::uint16_t map_y_c(double cell) {
  return project(cell, kScaleY, digi::penhid::kExtentY);
}

inline std::uint64_t profile_mean(const Profile& p) {
  if (!p.full()) return 0;
  std::uint64_t s = 0;
  for (const std::uint32_t v : p.col) s += v;
  for (const std::uint32_t v : p.row) s += v;
  return s / (kDimCols + kDimRows);
}

struct PenFeed {
  digi::penhid::PenReport report;
  bool have_judgement = false;
  // Where the position came from: 'c' profiles, 'p' controller cell,
  // 'h' held over, '?' not yet.
  char route = '?';
  pressure::Decoder pressure;
  PenPosition position;
  double cell_x = -1.0, cell_y = -1.0;

  // True when the frame had full profiles and the report was updated.
  bool update(const Frame& f) {
    if (f.have_pen_state)
      report.scan_time = std::uint16_t(f.pen_timestamp & 0xFFFFu);
    for (const SubFrame& sf : f.released) pressure.feed(sf);
    if (!f.have_profile || !f.profile.full()) return false;
    const std::uint64_t pm = profile_mean(f.profile);
    report.in_range = pm > kPresencePm;
    report.tip = pm >= kTipPm;
    if (!report.tip) pressure.lift();
    report.pressure = report.tip ? pressure.report() : 0;

    position = locate_pen(f.profile);
    if (position.valid()) {
      report.x = map_x_c(position.col.cell);
      report.y = map_y_c(position.row.cell);
      route = 'c';
      cell_x = position.col.cell;
      cell_y = position.row.cell;
    } else if (f.profile.refined_col != 0xFF && f.profile.refined_row != 0xFF) {
      report.x = map_x(f.profile.refined_col);
      report.y = map_y(f.profile.refined_row);
      route = 'p';
      cell_x = double(f.profile.refined_col);
      cell_y = double(f.profile.refined_row);
    } else {
      route = 'h';
    }
    have_judgement = true;
    return true;
  }
};

}  // namespace feed
}  // namespace pen

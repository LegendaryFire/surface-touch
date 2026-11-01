// SPDX-License-Identifier: MIT
// Pen feed policy — from decoded frame state to the pen front's report
// fields.
//
//  * InRange / TipSwitch: thresholds on the mean of the 0x5B profiles.
//    Presence band [0.5, 70]; tip cut below. No debouncing and no
//    temporal filter: the threshold is the whole mechanism, held across
//    frames that carry no full 0x5B.
//  * X/Y: the profile centroid (pen_position.hpp) through map_x_c/map_y_c.
//    The controller's refined cell through map_x/map_y is the fallback when a
//    profile axis carries no signal; with neither, the last position is held.
//  * ScanTime: u16 of the 0x57 timestamp; held across frames without a 0x57.
//  * TipPressure: decoded from the mode-11 sub-frames (pen_pressure.hpp) and
//    reported while TipSwitch is set; 0 otherwise, with the smoother
//    restarted.
//  * BarrelSwitch / Invert / Eraser / Tilt: PenReport's constants
//    (pen_uhid.hpp) — the wire carries no data for them.
#pragma once

#include <cstdint>

#include "pen_frame.hpp"
#include "pen_position.hpp"
#include "pen_pressure.hpp"
#include "pen_uhid.hpp"

namespace pen {
namespace feed {

// Predicate cuts. Presence sits inside the empty band [0.5, 70]; held
// contacts never approach it (minimum 2047).
inline constexpr std::uint64_t kPresencePm = 50;    // band [0.5, 70]
// Tip: the empty band between hover (max 996) and held contact (min 2047) is
// (996, 2047); the cut is its midpoint.
inline constexpr std::uint64_t kTipPm = 1521;       // band (996, 2047)

// Integer-cell map for the refined-cell fallback: grid 78x52 -> descriptor
// logical extents, anchored at x cells 0.75..76.00 and y cells 1.00..50.25.
// The quadrupled-integer form keeps the map exact: x = (4c-3)·9600/301,
// y = (4r-4)·7200/197, clamped to the declared logical range.
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

// Continuous cell->logical map for the centroid route:
// logical = trunc(cell * scale + 0.5) per axis, no offset term. The centroid
// is in cell-index units (cell i centred at i), and the scale places the
// centres of the outer cells 0 and kDim-1 on the declared extent's ends:
// scale = kExtent / (kDim - 1).
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

// Mean over the column + row profiles (130 cells); 0 for a half-length
// profile, which carries no vectors.
inline std::uint64_t profile_mean(const Profile& p) {
  if (!p.full()) return 0;
  std::uint64_t s = 0;
  for (const std::uint32_t v : p.col) s += v;
  for (const std::uint32_t v : p.row) s += v;
  return s / (kDimCols + kDimRows);
}

// Stateful per-pen judge. update() refreshes ScanTime from every frame's
// 0x57 and re-judges InRange/TipSwitch/position on frames carrying a full
// 0x5B; every other frame holds the previous judgement whole.
struct PenFeed {
  digi::penhid::PenReport report;
  bool have_judgement = false;  // at least one full-profile frame judged
  // Which route update() used for x/y, and the cell it came from before the
  // map's clamp: 'c' centroid, 'p' refined cell, 'h' hold (x/y kept), '?'
  // nothing judged yet. For traces and calibration tools; no report field
  // reads these.
  char route = '?';
  pressure::Decoder pressure;
  PenPosition position;  // the last full-profile frame's estimate
  double cell_x = -1.0, cell_y = -1.0;

  // Returns true when this frame was judged (a full 0x5B arrived): one
  // report per profile frame, as on the touch front.
  bool update(const Frame& f) {
    if (f.have_pen_state)  // ScanTime: the 0x57 timestamp, truncated to u16
      report.scan_time = std::uint16_t(f.pen_timestamp & 0xFFFFu);
    for (const SubFrame& sf : f.released) pressure.feed(sf);
    if (!f.have_profile || !f.profile.full()) return false;
    const std::uint64_t pm = profile_mean(f.profile);
    report.in_range = pm > kPresencePm;
    report.tip = pm >= kTipPm;  // pm monotone: tip ⊂ in_range by the bands
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
      route = 'h';  // x/y and cell_x/y hold the previous judgement
    }
    have_judgement = true;
    return true;
  }
};

}  // namespace feed
}  // namespace pen

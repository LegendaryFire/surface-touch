// SPDX-License-Identifier: MIT
// Sub-cell pen position from the 0x5B axis profiles.
//
// Each profile holds one signal-power value per sensor cell along its axis.
// The pen shows up as one sharp peak a few cells wide. Per axis:
//
//   1. amplitude = sqrt(power)
//   2. the run: the connected cells around the strongest one whose amplitude
//      is at least kRunThreshold of the peak (cv::connectedComponents)
//   3. position = centroid of (amplitude - threshold) over the run
//      (cv::moments)
//
// Subtracting the threshold keeps distant noise cells from pulling the
// centroid, and lets the weight of the run's fringe cells fall smoothly to
// zero, so the estimate moves continuously between cells. The result is in
// cell-index units: cell i is centred at i.
//
// The threshold keeps the sub-cell fraction evenly spread (no snapping
// towards cell centres) without adding frame-to-frame jitter.
#pragma once

#include <cstdint>
#include <vector>

#include "pen_frame.hpp"

namespace pen {

inline constexpr double kRunThreshold = 0.10;  // of the peak amplitude

struct AxisPosition {
  bool valid = false;  // false: the profile carries no signal
  double cell = 0.0;   // centroid, cell-index units
  int peak = -1;       // strongest cell
  int first = -1, last = -1;  // the run's extent (inclusive)
};

struct PenPosition {
  AxisPosition col, row;
  bool valid() const { return col.valid && row.valid; }
};

AxisPosition locate_on_axis(const std::vector<std::uint32_t>& profile);

// Both axes of a full profile; invalid for a half-length one.
PenPosition locate_pen(const Profile& p);

}  // namespace pen

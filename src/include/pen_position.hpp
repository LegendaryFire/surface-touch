// SPDX-License-Identifier: MIT
// Pen position finer than one cell, from the 0x5B axis profiles. On each
// axis the pen is a peak a few cells wide; the position is the centroid of
// the cells around the peak, weighted by how far they rise above 10% of it.
// The result is in cells, with cell i centred at i.
#pragma once

#include <cstdint>
#include <vector>

#include "pen_frame.hpp"

namespace pen {

inline constexpr double kRunThreshold = 0.10;

struct AxisPosition {
  bool valid = false;
  double cell = 0.0;
  int peak = -1;
  int first = -1, last = -1;
};

struct PenPosition {
  AxisPosition col, row;
  bool valid() const { return col.valid && row.valid; }
};

AxisPosition locate_on_axis(const std::vector<std::uint32_t>& profile);

PenPosition locate_pen(const Profile& p);

}  // namespace pen

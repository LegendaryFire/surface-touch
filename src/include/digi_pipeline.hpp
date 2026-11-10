// SPDX-License-Identifier: MIT
// Touchscreen contact engine: the shared tracker run on the digitizer's
// matrix frames.
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "digi_frame.hpp"
#include "touch_tracker.hpp"

namespace digi {

// This digitizer's 0x06 values, turned into thresholds the same way as the
// pad's.
struct DeviceConfig {
  int rows = kRows;
  int cols = kCols;
  std::uint32_t him_w = 30348;  // 0.01 mm
  std::uint32_t him_h = 20232;

  std::uint8_t gate_lo = 172;
  std::uint8_t gate_hi = 182;     // block 0 max, not an edge
  std::uint8_t gate = 180;
  std::uint8_t baseline = 180;
  std::uint8_t spawn_gate = 100;
  std::uint8_t core_join = 120;   // cores joined through cells <= this are one contact
  double palm_mm2_per_core = 700.0;
};

using Blob = touch::Blob;
using Track = touch::Track;

// Position in 0.01 mm.
struct Contact {
  std::uint16_t x;
  std::uint16_t y;
  std::uint8_t slot;
  bool confidence;
  bool in_contact;
};

// One per finger collection in the touchscreen descriptor.
inline constexpr int kWireIds = 5;

class DigiEngine final {
 public:
  explicit DigiEngine(DeviceConfig cfg = {});

  // While pen_near is set every touch is a palm: the hand resting on the
  // screen while writing.
  std::vector<Contact> process(const std::uint8_t* cells, bool pen_near = false);

  // Lift reports for the tracks the last process() call retired.
  std::vector<Contact> drain_releases() { return std::exchange(releases_, {}); }

 private:
  Contact to_contact(double x, double y, std::uint8_t wire, bool palm,
                     bool in_contact) const;

  DeviceConfig cfg_;
  touch::ContactTracker tracker_;
  std::vector<Contact> releases_;
};

}  // namespace digi

// SPDX-License-Identifier: MIT
// Digitizer contact engine: the shared touch pipeline (touch_tracker.hpp)
// driven by the digitizer's matrix frames, reporting contacts in himetric.
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "digi_frame.hpp"
#include "touch_tracker.hpp"

namespace digi {

// Defaults are this digitizer's 0x06 DeviceParameters response, with the
// thresholds derived as heat::DeviceParams::to_config does for the pad:
//   gate_lo = trunc(avg + 4*(min-avg) + 0.5) = 172   (block 0)
//   gate    = trunc(avg)                     = 180   (block 0, idle cell)
//   spawn   = trunc(avg)                     = 100   (block 2)
struct DeviceConfig {
  int rows = kRows;
  int cols = kCols;
  std::uint32_t him_w = 30348;  // 303.48 mm
  std::uint32_t him_h = 20232;  // 202.32 mm

  std::uint8_t gate_lo = 172;     // OFF->ON edge
  std::uint8_t gate_hi = 182;     // block 0 max, not an edge
  std::uint8_t gate = 180;        // ON->OFF edge
  std::uint8_t baseline = 180;    // blob weight = baseline - cell
  std::uint8_t spawn_gate = 100;  // ON cells at or below this are cores
  std::uint8_t core_join = 120;   // cores joined through cells <= this are
                                  // one contact
  // Palm rejection threshold, ON area per core (touch::SensorConfig).
  // Palms are reported with Confidence = 0.
  double palm_mm2_per_core = 700.0;
};

using Blob = touch::Blob;
using Track = touch::Track;

// x/y in the device's himetric extent (0.01 mm), clamped to him_w/him_h; the
// front's logical axes use this domain 1:1.
struct Contact {
  std::uint16_t x;
  std::uint16_t y;
  std::uint8_t slot;  // wire id (Track::wire)
  bool confidence;
  bool in_contact;    // false only on a retire-frame release
};

// Contact-id pool, fixed by the five Finger collections in digi_uhid.hpp.
inline constexpr int kWireIds = 5;

class DigiEngine final {
 public:
  explicit DigiEngine(DeviceConfig cfg = {});

  // One matrix frame in, live contact set out (may be empty). Scene frames
  // carry no cell grid and are not passed here. While pen_near is set, every
  // touch is a palm: the hand resting on the screen while the pen writes.
  std::vector<Contact> process(const std::uint8_t* cells, bool pen_near = false);

  // One Contact{slot, in_contact=false} per track retired by the last
  // process() call.
  std::vector<Contact> drain_releases() { return std::exchange(releases_, {}); }

 private:
  Contact to_contact(double x, double y, std::uint8_t wire, bool palm,
                     bool in_contact) const;

  DeviceConfig cfg_;
  touch::ContactTracker tracker_;
  std::vector<Contact> releases_;
};

}  // namespace digi

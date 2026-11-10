// SPDX-License-Identifier: MIT
// Touchpad contact engine: the shared tracker run on the pad's heat frames.
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "engine.hpp"
#include "touch_tracker.hpp"

namespace heat {

// This pad's own 0x06 values; heatd reads the live ones at start.
struct DeviceConfig {
  int rows = 18;
  int cols = 26;
  // Extent in 0.01 mm (120 x 80 mm).
  std::uint32_t him_w = 12000;
  std::uint32_t him_h = 8000;
  // Idle cells read 180; a finger pulls them down. A cell turns on below
  // gate_lo and off again at gate, with a dead band in between.
  std::uint8_t gate_lo = 172;
  std::uint8_t gate_hi = 182;   // block max, not the release edge
  std::uint8_t gate = 180;
  std::uint8_t baseline = 180;  // signal weight w = baseline - cell
  // Cells at or below this start a contact. The device's touch-signal
  // average, well below the fixed edge pattern, so that pattern never does.
  std::uint8_t spawn_gate = 100;
  double palm_mm2_per_core = 700.0;
};

using Blob = touch::Blob;
using Track = touch::Track;

class HeatEngine final : public Engine {
 public:
  explicit HeatEngine(DeviceConfig cfg = {});

  std::vector<Contact> process(const Frame& f) override;

  const std::vector<Blob>& last_blobs() const { return tracker_.blobs(); }
  int region_count() const { return tracker_.region_count(); }
  const touch::ContactTracker& tracker() const { return tracker_; }
  const std::vector<Track>& last_tracks() const { return tracker_.tracks(); }
  bool button() const override { return button_; }

  // Lift reports for the tracks the last process() call retired.
  std::vector<Contact> drain_releases() {
    return std::exchange(releases_, {});
  }

 private:
  // The descriptor's contact id range, 0..5.
  static constexpr int kWireIds = 6;

  DeviceConfig cfg_;
  touch::ContactTracker tracker_;
  bool button_ = false;
  std::vector<Contact> releases_;
};

}  // namespace heat

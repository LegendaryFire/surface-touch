// SPDX-License-Identifier: MIT
// Pad contact engine: the shared touch pipeline (touch_tracker.hpp) driven by
// the pad's heat frames, reporting contacts in PTP logical units.
//
// DeviceConfig defaults are this machine's live-read values; devparams.hpp
// reads the live 0x06 response (or a saved one via heatd --device-params)
// and overrides them at daemon start.
#pragma once

#include <cstdint>
#include <utility>  // std::exchange (drain_releases)
#include <vector>

#include "engine.hpp"
#include "touch_tracker.hpp"

namespace heat {

// ---- device-supplied configuration (the 0x06 feature report) ---------------
// The defaults below are this pad's own values, used when the live read
// fails.
struct DeviceConfig {
  // rows=18, cols=26, product 468 = heatmap length.
  int rows = 18;
  int cols = 26;
  // himetric extents, 0.01 mm units (12000 x 8000 = 120.00 x 80.00 mm).
  std::uint32_t him_w = 12000;
  std::uint32_t him_h = 8000;
  // Cells-on band from the AC/DC baseline block {min 178, max 182, avg 180};
  // idle cell 0xB4 == avg == 180. A finger DECREASES the cell value
  // (deviations 62..176 breach downward):
  //   OFF->ON at  cell <  gate_lo (172)   — downward breach of the entry edge
  //   ON ->OFF at cell >= gate    (180)   — return to baseline releases
  //   in [gate_lo, gate): state held      — the hysteresis dead band
  // gate_lo is not the raw min: it is trunc(avg + 4*(min-avg) + 0.5) = 172
  // (devparams.hpp::to_config). Entering at the raw min (178) latched the
  // hover band 172..177 and merged adjacent fingers. gate_hi (182) is the
  // block's max, kept for the live override path; release happens on the
  // return to baseline.
  std::uint8_t gate_lo = 172;   // OFF->ON edge
  std::uint8_t gate_hi = 182;   // block max, not the release edge
  std::uint8_t gate = 180;      // ON->OFF edge (baseline avg)
  std::uint8_t baseline = 180;  // signal weight w = baseline - cell
  // Contact-spawn strength gate: a cell is a CORE (tracker-visible blob)
  // iff cell <= spawn_gate. Two passes share one label image: the strict
  // pass separates strong cores, the loose pass (the ON map) fills them.
  // The value is the device's TouchSignal average (block 2 avg = 100 raw on
  // this machine). The fixed edge/electrode pattern stays well above it, so
  // the phantom scroll pattern never forms a core, while close fingers with
  // a shallow saddle separate into two cores. A core that disappears (lift) retires through the tracker's
  // miss count; the hysteresis layer does not hold contacts alive below the
  // touch level.
  std::uint8_t spawn_gate = 100;  // core edge: cell <= gate is a core
};

using Blob = touch::Blob;
using Track = touch::Track;

class HeatEngine final : public Engine {
 public:
  explicit HeatEngine(DeviceConfig cfg = {});

  std::vector<Contact> process(const Frame& f) override;

  // Engine state, for inspection.
  const std::vector<Blob>& last_blobs() const { return tracker_.blobs(); }
  int region_count() const { return tracker_.region_count(); }
  const std::vector<Track>& last_tracks() const { return tracker_.tracks(); }
  bool button() const override { return button_; }

  // One TipSwitch=0 entry per track retired by the last process() call, keyed
  // by its wire id. Drain once per frame, right after process().
  std::vector<Contact> drain_releases() {
    return std::exchange(releases_, {});
  }

 private:
  // Wire-id pool: the descriptor's ContactIdentifier range (3 b, 0..5).
  static constexpr int kWireIds = 6;

  DeviceConfig cfg_;
  touch::ContactTracker tracker_;
  bool button_ = false;  // last frame's press state (kPhysStateButton)
  std::vector<Contact> releases_;
};

}  // namespace heat

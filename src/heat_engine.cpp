// SPDX-License-Identifier: MIT
#include "heat_engine.hpp"

#include <algorithm>

namespace heat {

namespace {
touch::SensorConfig sensor(const DeviceConfig& c) {
  touch::SensorConfig s;
  s.rows = c.rows;
  s.cols = c.cols;
  s.him_w = c.him_w;
  s.him_h = c.him_h;
  s.gate_lo = c.gate_lo;
  s.gate = c.gate;
  s.baseline = c.baseline;
  s.spawn_gate = c.spawn_gate;
  s.core_join = c.spawn_gate;
  s.palm_mm2_per_core = c.palm_mm2_per_core;
  return s;
}

// 0.01 mm to 0..4095.
std::pair<std::uint16_t, std::uint16_t> to_ptp_xy(double x, double y,
                                                  const DeviceConfig& cfg) {
  const std::int64_t hx = std::int64_t(x), hy = std::int64_t(y);
  const std::int64_t px = std::clamp<std::int64_t>(hx * 4095 / cfg.him_w, 0, 4095);
  const std::int64_t py = std::clamp<std::int64_t>(hy * 4095 / cfg.him_h, 0, 4095);
  return {std::uint16_t(px), std::uint16_t(py)};
}
}  // namespace

HeatEngine::HeatEngine(DeviceConfig cfg)
    : cfg_(cfg), tracker_(sensor(cfg), kWireIds) {}

std::vector<Contact> HeatEngine::process(const Frame& f) {
  button_ = (f.phys & kPhysStateButton) != 0;
  tracker_.process(f.cells);

  releases_.clear();
  for (const touch::Retired& d : tracker_.retired()) {
    const auto [px, py] = to_ptp_xy(d.x, d.y, cfg_);
    releases_.push_back(Contact{px, py, d.wire, /*confidence=*/!d.palm,
                                /*in_contact=*/false});
  }

  const auto& tracks = tracker_.tracks();
  const std::size_t n = std::min(tracks.size(), kMaxContacts);
  std::vector<Contact> out;
  out.reserve(n);
  for (std::size_t t = 0; t < n; ++t) {
    const auto [px, py] = to_ptp_xy(tracks[t].x, tracks[t].y, cfg_);
    out.push_back(Contact{px, py, tracks[t].wire, /*confidence=*/!tracks[t].palm,
                          /*in_contact=*/true});
  }
  return out;
}

}  // namespace heat

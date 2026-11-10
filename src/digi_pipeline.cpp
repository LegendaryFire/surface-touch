// SPDX-License-Identifier: MIT
#include "digi_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace digi {

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
  s.core_join = c.core_join;
  s.palm_mm2_per_core = c.palm_mm2_per_core;
  return s;
}
}  // namespace

DigiEngine::DigiEngine(DeviceConfig cfg)
    : cfg_(cfg), tracker_(sensor(cfg), kWireIds) {}

std::vector<Contact> DigiEngine::process(const std::uint8_t* cells, bool pen_near) {
  tracker_.process(cells);
  if (pen_near) tracker_.mark_all_palm();

  releases_.clear();
  for (const touch::Retired& d : tracker_.retired())
    releases_.push_back(to_contact(d.x, d.y, d.wire, d.palm, /*in_contact=*/false));

  std::vector<Contact> out;
  out.reserve(tracker_.tracks().size());
  for (const Track& tr : tracker_.tracks())
    out.push_back(to_contact(tr.x, tr.y, tr.wire, tr.palm, /*in_contact=*/true));
  return out;
}

// Rounded and clamped to the screen.
Contact DigiEngine::to_contact(double x, double y, std::uint8_t wire, bool palm,
                               bool in_contact) const {
  const std::int64_t hx = std::llround(x), hy = std::llround(y);
  const auto px = std::uint16_t(std::clamp<std::int64_t>(hx, 0, cfg_.him_w));
  const auto py = std::uint16_t(std::clamp<std::int64_t>(hy, 0, cfg_.him_h));
  return Contact{px, py, wire, /*confidence=*/!palm, in_contact};
}

}  // namespace digi

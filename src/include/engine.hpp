// SPDX-License-Identifier: MIT
// Contact engine interface: one heatmap frame in, a contact set out. The
// heat engine (heat_engine.hpp) implements it in stages: threshold,
// segmentation, tracking, classification, packing.
#pragma once

#include <cstdint>
#include <vector>

#include "frame.hpp"

namespace heat {

struct Contact {
  std::uint16_t x;      // 0..4095 PTP scale
  std::uint16_t y;      // 0..4095
  std::uint8_t slot;    // PTP tracking slot
  bool confidence;      // finger confirmed (classifier)
  bool in_contact;      // vs hover (hover not implemented)
};

// The uhid front declares 5 contact slots (contact_max);
// the engine never reports more, and the 0x81 packer always writes all 5
// slot entries so the trailing fields keep their fixed bit offsets.
inline constexpr std::size_t kMaxContacts = 5;

// Engine interface.
class Engine {
 public:
  virtual ~Engine() = default;
  // One heatmap frame in, contact set out (may be empty). ts16 forwarding is
  // the engine's duty for the PTP scan-time field.
  virtual std::vector<Contact> process(const Frame& f) = 0;
  // The pad's press state for the last processed frame: the
  // engine reads it from the frame, the packer writes it as PTP Button 1.
  // Default false for an engine without one.
  virtual bool button() const { return false; }
};

// The gate-closed engine shipped in phase 5: emits nothing, keeps cadence.
class NullEngine final : public Engine {
 public:
  std::vector<Contact> process(const Frame&) override { return {}; }
};

}  // namespace heat

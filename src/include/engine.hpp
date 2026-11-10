// SPDX-License-Identifier: MIT
// Contact engine interface: one heat frame in, a set of contacts out.
#pragma once

#include <cstdint>
#include <vector>

#include "frame.hpp"

namespace heat {

struct Contact {
  std::uint16_t x;  // 0..4095
  std::uint16_t y;  // 0..4095
  std::uint8_t slot;
  bool confidence;
  bool in_contact;
};

// The touchpad device declares 5 contacts.
inline constexpr std::size_t kMaxContacts = 5;

class Engine {
 public:
  virtual ~Engine() = default;
  virtual std::vector<Contact> process(const Frame& f) = 0;
  // Whether the pad was pressed in the last frame.
  virtual bool button() const { return false; }
};

// Reports nothing.
class NullEngine final : public Engine {
 public:
  std::vector<Contact> process(const Frame&) override { return {}; }
};

}  // namespace heat

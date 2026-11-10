// SPDX-License-Identifier: MIT
// Packs the touchpad's input report 0x81, 29 bytes:
//   5 x { Confidence 1b, TipSwitch 1b, ContactId 3b, pad 3b, X 16b, Y 16b }
//   ScanTime 16b, ContactCount 8b, Button 1b, pad 7b
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "engine.hpp"
#include "wire.hpp"

namespace heat {

using wire::put_bits;

// All 5 contact slots are always written, unused ones as zeros, so the fields
// after them stay at fixed offsets.
inline void pack_report_0x81(std::uint8_t* out, std::size_t payload_bytes,
                             std::uint16_t scan_time,
                             const std::vector<Contact>& contacts,
                             bool button = false) {
  for (std::size_t i = 0; i < payload_bytes; ++i) out[i] = 0;
  std::size_t bit = 0;
  for (std::size_t slot = 0; slot < kMaxContacts; ++slot) {
    const Contact* c = slot < contacts.size() ? &contacts[slot] : nullptr;
    put_bits(out, bit, c ? ((c->confidence ? 1u : 0u) | (c->in_contact ? 2u : 0u)) : 0u, 2);
    bit += 2;
    put_bits(out, bit, c ? c->slot : 0u, 3);
    bit += 3;
    put_bits(out, bit, 0u, 3);
    bit += 3;
    put_bits(out, bit, c ? c->x : 0u, 16);
    bit += 16;
    put_bits(out, bit, c ? c->y : 0u, 16);
    bit += 16;
  }
  put_bits(out, bit, scan_time, 16);
  bit += 16;
  put_bits(out, bit, static_cast<std::uint32_t>(contacts.size()), 8);
  bit += 8;
  put_bits(out, bit, button ? 1u : 0u, 1);
  bit += 1;
  put_bits(out, bit, 0u, 7);
}

}  // namespace heat

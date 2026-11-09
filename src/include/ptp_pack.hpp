// SPDX-License-Identifier: MIT
// PTP 0x81 touchpad input report packing.
//
// Layout walked from the device's raw report descriptor: 29 input items,
// 232 bits total.
//   per contact (40 b): Confidence(1b, U:000D:0047) + TipSwitch(1b, U:000D:0042)
//     | ContactIdentifier(3b, U:000D:0051, 0..5) | padding(3b) |
//     X(16b, U:0001:0030, 0..4095) | Y(16b, U:0001:0031, 0..4095)
//   x5 = 200 b
//   + TransducerIndex(16b, U:000D:0056) @bit200 + ContactCount(8b, U:000D:0054) @216
//   + Button(1b, U:0009:0001) @224 + padding(7b)                 = 232 b = 29 B
// (Descriptor offsets 610..1026; the fixture collection agrees.)
// Little-endian bit packing per HID 6.2.2.2 — LSB of each item is the next
// bit on the wire.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "engine.hpp"
#include "wire.hpp"

namespace heat {

using wire::put_bits;

// Packs one 0x81 report (payload only, report-id byte excluded — same
// convention as the descriptor bit totals). The 5 contact slots declared by
// the descriptor are ALWAYS written (unused slots zero), so the trailing
// ScanTime/ContactCount fields sit at their fixed bits 200/216 regardless of
// how many contacts are active — matching how the slot array's Logical
// Minimum 0 is defined to work. `contacts.size()` must be ≤ kMaxContacts
// (fixture contact_max = 5); Contact.slot is the per-contact
// ContactIdentifier assigned by the engine at spawn (stable for the
// contact's life) — not the array position.
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
    put_bits(out, bit, c ? c->slot : 0u, 3);  // ContactId
    bit += 3;
    put_bits(out, bit, 0u, 3);                // padding
    bit += 3;
    put_bits(out, bit, c ? c->x : 0u, 16);
    bit += 16;
    put_bits(out, bit, c ? c->y : 0u, 16);
    bit += 16;
  }
  put_bits(out, bit, scan_time, 16);  // U:000D:0056 (forwarded ts16)
  bit += 16;
  put_bits(out, bit, static_cast<std::uint32_t>(contacts.size()), 8);  // U:000D:0054
  bit += 8;
  // U:0009:0001 Button 1 = the pad's own press state, forwarded from frame
  // byte 635 by the engine.
  put_bits(out, bit, button ? 1u : 0u, 1);
  bit += 1;
  put_bits(out, bit, 0u, 7);          // padding
}

}  // namespace heat

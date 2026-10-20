// SPDX-License-Identifier: MIT
// Minimal HID descriptor walk: compute per-report-id bit totals.
//
// Used by the uhid front to size reports from the byte-exact fixture
// (fixtures/ptp_collection_000D_0005.bin, a slice of the device's own report
// descriptor), so report sizes are derived, never hand-counted.
#pragma once

#include <cstdint>
#include <map>
#include <vector>

namespace heat {

struct ReportBits {
  std::uint32_t input = 0;
  std::uint32_t output = 0;
  std::uint32_t feature = 0;
  // per-report usage occurrence counts, key = (page << 16) | usage
  std::map<std::uint64_t, unsigned> usages;
};

inline std::uint64_t usage_key(std::uint32_t page, std::uint32_t usage) {
  return (std::uint64_t(page) << 16) | usage;
}

// Returns id -> bit totals (payload only, id byte excluded), plus the total
// descriptor length for sanity. Tolerates the collections inside the fixture.
inline std::map<std::uint16_t, ReportBits> report_bits(const std::uint8_t* d,
                                                       std::size_t n) {
  std::map<std::uint16_t, ReportBits> out;
  std::uint16_t id = 0;
  std::uint32_t size = 0, count = 0;
  std::uint32_t page = 0;
  std::vector<std::uint64_t> pending;  // usages declared since last main item
  unsigned i = 0;
  while (i < n) {
    std::uint8_t head = d[i];
    unsigned sz = head & 0x03;
    if (sz == 3) sz = 4;
    if (head == 0xFE) {  // long item: ONLY the 0xFE header byte (spec 6.2.2.3)
      i += 2 + (i + 1 < n ? d[i + 1] : 0);
      continue;
    }
    auto byte_at = [&](unsigned k) -> std::uint32_t {
      return i + 1 + k < n ? d[i + 1 + k] : 0u;
    };
    std::uint32_t data = 0;
    for (unsigned k = 0; k < sz; ++k) data |= byte_at(k) << (8 * k);
    switch (head & 0xFC) {
      case 0x04:  // Usage Page
        page = data;
        break;
      case 0x08:  // Usage (accumulated until the next main item)
        pending.push_back(usage_key(page, data));
        break;
      case 0x84:  // Report ID
        id = static_cast<std::uint16_t>(data);
        break;
      case 0x74:  // Report Size
        size = data;
        break;
      case 0x94:  // Report Count
        count = data;
        break;
      case 0xA0:  // Collection consumes its declared Usage(s) (spec
        pending.clear();  // 6.2.2.4) — leaking e.g. U:0022 into the next
        break;            // main item's attribution was the original bug
      case 0x80:  // Input
      case 0x90:  // Output
      case 0xB0: { // Feature
        ReportBits& rb = out[id];
        std::uint32_t bits = size * count;
        const std::uint8_t typ = static_cast<std::uint8_t>(head & 0xFC);
        if (typ == 0x80) rb.input += bits;
        else if (typ == 0x90) rb.output += bits;
        else if (typ == 0xB0) rb.feature += bits;
        // Usage attribution (HID 6.2.2.6): 0 pending -> Constant padding
        // (skip); 1 pending carries the whole count; multiple usages share
        // the count in declaration order (first rem get +1).
        if (!pending.empty()) {
          std::uint32_t k = static_cast<std::uint32_t>(pending.size());
          std::uint32_t base = count / k, rem = count % k, left = count;
          for (size_t j = 0; j < pending.size() && left > 0; ++j) {
            std::uint32_t take = base + (j < rem ? 1u : 0u);
            if (take == 0) take = 1;  // count < #usages: first get one field
            take = std::min(take, left);
            rb.usages[pending[j]] += take;
            left -= take;
          }
        }
        pending.clear();
        break;
      }
      default:  // EndCollection/other globals: usages belong to the next
        break;  // main item, keep pending
    }
    i += 1 + sz;
  }
  return out;
}

}  // namespace heat

// SPDX-License-Identifier: MIT
// Little-endian reads and LSB-first bit packing for HID reports.
#pragma once

#include <endian.h>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace wire {

inline std::uint16_t le16(const std::uint8_t* p) {
  std::uint16_t v;
  std::memcpy(&v, p, sizeof v);
  return le16toh(v);
}

inline std::uint32_t le32(const std::uint8_t* p) {
  std::uint32_t v;
  std::memcpy(&v, p, sizeof v);
  return le32toh(v);
}

inline float le_f32(const std::uint8_t* p) { return std::bit_cast<float>(le32(p)); }

// Writes the low n bits of v at bit `at`, leaving the other bits alone.
inline void put_bits(std::uint8_t* p, std::size_t at, std::uint32_t v, unsigned n) {
  for (unsigned i = 0; i < n; ++i) {
    const std::size_t bit = at + i;
    const auto mask = std::uint8_t(1u << (bit & 7u));
    if ((v >> i) & 1u)
      p[bit >> 3] |= mask;
    else
      p[bit >> 3] &= std::uint8_t(~mask);
  }
}

}  // namespace wire

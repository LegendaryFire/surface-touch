// SPDX-License-Identifier: MIT
// The pad front's report descriptor: fixtures/ptp_collection_000D_0005.bin,
// a byte-exact slice of the device's own report descriptor, embedded at
// compile time so the front can never drift from it.
#pragma once

#include <cstddef>
#include <cstdint>

namespace ptp {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++26-extensions"  // #embed is standard from C++26
inline constexpr std::uint8_t collection[] = {
#embed "../fixtures/ptp_collection_000D_0005.bin"
};
#pragma GCC diagnostic pop

inline constexpr std::size_t collection_size = sizeof collection;

}  // namespace ptp

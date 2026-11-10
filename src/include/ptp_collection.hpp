// SPDX-License-Identifier: MIT
// The virtual touchpad's report descriptor: the touchpad collection copied
// byte for byte from the device's own descriptor.
#pragma once

#include <cstddef>
#include <cstdint>

namespace ptp {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++26-extensions"
inline constexpr std::uint8_t collection[] = {
#embed "../fixtures/ptp_collection_000D_0005.bin"
};
#pragma GCC diagnostic pop

inline constexpr std::size_t collection_size = sizeof collection;

}  // namespace ptp

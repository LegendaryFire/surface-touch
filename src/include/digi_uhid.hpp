// SPDX-License-Identifier: MIT
// The virtual touchscreen: a standard Windows 8 multi-touch Touch Screen,
// with axes in 0.01 mm over the digitizer's full extent.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "digi_pipeline.hpp"
#include "wire.hpp"

namespace digi {
namespace uhid {

inline constexpr std::uint8_t kReportInput = 0x0C;
inline constexpr std::uint8_t kReportCaps = 0x0D;
inline constexpr std::size_t kInputPayloadBytes = 39;

inline constexpr unsigned kFrontContacts = 5;

inline constexpr std::uint16_t kExtentX = 30348;
inline constexpr std::uint16_t kExtentY = 20232;

inline constexpr std::uint8_t kRdesc[] = {
    0x05, 0x0D, 0x09, 0x04, 0xA1, 0x01,  // App (Digitizer:Touch Screen)
    0x85, 0x0C,                          // Report ID 0x0C
    0x95, 0x01, 0x75, 0x08, 0x26, 0xFF, 0x00, 0x15, 0x00,
    0x81, 0x03,                          // Input Cnst 8b
    0x09, 0x54, 0x81, 0x02,              // Input ContactCount u8

#define DIGI_CONTACT_COLLECTION                                                    \
  0x05, 0x0D, 0x09, 0x22, 0xA1, 0x02,    /* Log (Digitizer:Finger)  */             \
    0x09, 0x42, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01,                    \
    0x81, 0x02,                          /* Tip Switch 1b          */              \
    0x81, 0x03,                          /* Cnst pad 1b            */              \
    0x09, 0x47, 0x81, 0x02,              /* Confidence 1b          */              \
    0x95, 0x05, 0x81, 0x03,              /* Cnst pad 5b            */              \
    0x09, 0x51, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0x81, 0x02, /* CntID u16 */ \
    0x05, 0x01, 0x75, 0x10, 0x95, 0x01,  /* GD page, u16           */              \
    0x55, 0x0D, 0x65, 0x11,              /* Unit 0.01 mm           */              \
    0x09, 0x30, 0x26, 0x8C, 0x76,        /* X, LogMax 30348        */              \
    0x35, 0x00, 0x46, 0x8C, 0x76,        /* PhysMin 0, PhysMax 30348 */            \
    0x81, 0x02,                                                            /* X  */ \
    0x46, 0x08, 0x4F, 0x09, 0x31, 0x26, 0x08, 0x4F, /* PhysMax 20232, Y, LMax 20232 */ \
    0x81, 0x02,                                                            /* Y  */ \
    0xC0,                                    /* End Collection           */
    DIGI_CONTACT_COLLECTION DIGI_CONTACT_COLLECTION DIGI_CONTACT_COLLECTION
    DIGI_CONTACT_COLLECTION DIGI_CONTACT_COLLECTION
#undef DIGI_CONTACT_COLLECTION

    0x05, 0x0D, 0x27, 0xFF, 0xFF, 0x00, 0x00, 0x75, 0x10, 0x95, 0x01,
    0x09, 0x56, 0x81, 0x02,              // Input ScanTime u16
    0x85, 0x0C,                          // Report ID 0x0C
    0x09, 0x55, 0x75, 0x08, 0x95, 0x04, 0x26, 0xFF, 0x00,
    0xB1, 0x02,                          // Feature Maximum Contact Count 4xB

    // Without this, hid-multitouch does not treat the device as Windows 8
    // multi-touch.
    0x06, 0x00, 0xFF, 0x09, 0xC6,        // Page(FF00) Usage(0xC6)
    0x75, 0x08, 0x95, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00,
    0x85, 0x0D, 0xB1, 0x02,              // Feature id 0x0D, 1 byte
    0xC0                                 // End Collection
};
inline constexpr std::size_t kRdescSize = sizeof(kRdesc);

// Every finger collection must carry the extents above.
namespace detail {
constexpr std::uint16_t le16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}
constexpr int count_item(const std::uint8_t* d, std::size_t n, std::uint8_t op,
                         std::uint16_t v) {
  int c = 0;
  for (std::size_t i = 0; i + 2 < n; ++i)
    if (d[i] == op && le16(d + i + 1) == v) ++c;
  return c;
}
}  // namespace detail
static_assert(detail::count_item(kRdesc, kRdescSize, 0x26, kExtentX) == kFrontContacts,
              "X logical maximum must be kExtentX in every collection");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x46, kExtentX) == kFrontContacts,
              "X physical maximum must be kExtentX in every collection");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x26, kExtentY) == kFrontContacts,
              "Y logical maximum must be kExtentY in every collection");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x46, kExtentY) == kFrontContacts,
              "Y physical maximum must be kExtentY in every collection");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x26, 30316) == 0,
              "the rev-1 wrong X value (0x766C) must not appear");

inline constexpr unsigned kVendor = 0x045E;
inline constexpr unsigned kProduct = 0x4D54;
inline constexpr const char* kName = "Surface Digitizer Touch";

using wire::put_bits;

// Unlike the touchpad report, Tip Switch comes before Confidence.
inline void pack_report_0x0C(std::uint8_t* out, std::size_t payload_bytes,
                             std::uint16_t scan_time,
                             const std::vector<Contact>& contacts) {
  for (std::size_t i = 0; i < payload_bytes; ++i) out[i] = 0;
  std::size_t bit = 8;
  put_bits(out, bit, static_cast<std::uint32_t>(contacts.size()), 8);
  bit += 8;
  for (unsigned slot = 0; slot < kFrontContacts; ++slot) {
    const Contact* c = slot < contacts.size() ? &contacts[slot] : nullptr;
    if (!c) { bit += 56; continue; }
    put_bits(out, bit + 0, c->in_contact ? 1u : 0u, 1);   // Tip Switch
    put_bits(out, bit + 2, c->confidence ? 1u : 0u, 1);   // Confidence
    put_bits(out, bit + 8, c->slot, 16);                  // Contact ID
    put_bits(out, bit + 24, c->x, 16);                    // X
    put_bits(out, bit + 40, c->y, 16);                    // Y
    bit += 56;
  }
  put_bits(out, bit, scan_time, 16);                      // Scan Time
}

// The reply must start with the report id: hid-multitouch drops one that
// doesn't and then allows 10 contacts instead of 5.
inline std::vector<std::uint8_t> feature_reply(std::uint8_t rid) {
  if (rid == kReportInput)
    return std::vector<std::uint8_t>{kReportInput,
                                     std::uint8_t(kFrontContacts),
                                     std::uint8_t(kFrontContacts),
                                     std::uint8_t(kFrontContacts),
                                     std::uint8_t(kFrontContacts)};
  if (rid == kReportCaps)
    return std::vector<std::uint8_t>{kReportCaps, 0};
  return {};
}

}  // namespace uhid
}  // namespace digi

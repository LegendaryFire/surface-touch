// SPDX-License-Identifier: MIT
// The digid uhid front: a multi-touch Touch Screen in the Windows 8 MT
// class, presenting the engine's contacts to the host.
//
//   * Grammar: the standard Windows 8 MT-class Touch Screen collection,
//     field set and order as that class defines them.
//   * Binding: hid-core.c/hid-multitouch.c (Linux 7.3) — everything the
//     descriptor must contain to reach MT_CLS_WIN_8 and parse correctly.
//   * Geometry: this device's 0x06 response — himW 30348 x himH 20232 in
//     0.01 mm; the front's logical axes are that domain 1:1, with no scale
//     factor.
// The pad's PTP front (src/main.cpp) works the same way: same uhid CREATE2
// shape, same GET_REPORT servicing, same release packing, same
// feature-derived report sizing via descwalk.hpp.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "digi_pipeline.hpp"

namespace digi {
namespace uhid {

// ---- report identity (template IDs kept: input and the MaxCount feature
// share report id 0x0C; the C6 capability marker gets its own id 0x0D) ----
inline constexpr std::uint8_t kReportInput = 0x0C;
inline constexpr std::uint8_t kReportCaps = 0x0D;
inline constexpr std::size_t kInputPayloadBytes = 39;  // 312 bits, no id byte

// Five contact collections, as the template and the pad's PTP declare; the
// engine's contact pool is fixed at this number.
inline constexpr unsigned kFrontContacts = 5;

// The device's declared extent, 0.01 mm (0x06 himW/himH).
inline constexpr std::uint16_t kExtentX = 30348;
inline constexpr std::uint16_t kExtentY = 20232;

// ---- the descriptor ----
// Byte for byte the template except the four range edits marked below and
// the appended C6 capability feature (which Linux needs to promote the
// device to the Win 8 MT group).
//
// Template items: App(Digitizer:TouchScreen)
// id 0x0C; Input Cnst 8b (usage inherited from the app usage — kept exactly);
// Input ContactCount 8b; five Log(Finger){ TipSwitch 1b, Cnst 1b, Confidence
// 1b, Cnst 5b, ContactID 16b (lmax 255), X 16b, Y 16b } with GD page and
// Unit 0x11; Input ScanTime 16b (lmax 65535 u32-form item); Feature
// MaxCount 4x8b (lmax 255) on the same id 0x0C.
//
// Edits (all of them):
//   * X:  Unit Exponent 0x0E->0x0D   (physical unit 0.1 mm -> 0.01 mm)
//         LogMax 10368 -> 30348, PhysMax 2592 -> 30348  (0x06 himW 1:1)
//   * Y:  LogMax 6912 -> 20232, PhysMax 1728 -> 20232   (0x06 himH 1:1)
//         (the template's logical:physical 4:1 ratio at 0.1 mm becomes 1:1 at
//         0.01 mm — same 0.025->0.01 mm step replaced by the native step; the
//         per-axis maxes are the firmware numbers, no rounding anywhere)
//   * appended: Page(FF00) Usage(0xC6) Feature id 0x0D, 1 byte — the exact
//         shape hid_scan_feature_usage() accepts (count 1, size 8) to promote
//         HID_GROUP_MULTITOUCH -> HID_GROUP_MULTITOUCH_WIN_8.
inline constexpr std::uint8_t kRdesc[] = {
    0x05, 0x0D, 0x09, 0x04, 0xA1, 0x01,  // App (Digitizer:Touch Screen)
    0x85, 0x0C,                          // Report ID 0x0C
    0x95, 0x01, 0x75, 0x08, 0x26, 0xFF, 0x00, 0x15, 0x00,
    0x81, 0x03,                          // Input Cnst 8b (padding: the app
                                         // usage was consumed by Collection)
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
    0x55, 0x0D, 0x65, 0x11,              /* UnitExp -3, Unit cm^1 -> 0.01 mm */    \
    0x09, 0x30, 0x26, 0x8C, 0x76,        /* X, LogMax 30348=0x768C (himW) */        \
    0x35, 0x00, 0x46, 0x8C, 0x76,        /* PhysMin 0, PhysMax 30348      */        \
    0x81, 0x02,                                                            /* X  */ \
    0x46, 0x08, 0x4F, 0x09, 0x31, 0x26, 0x08, 0x4F, /* PhysMax 20232, Y, LMax 20232 */ \
    0x81, 0x02,                                                            /* Y  */ \
    0xC0,                                    /* End Collection           */
    DIGI_CONTACT_COLLECTION DIGI_CONTACT_COLLECTION DIGI_CONTACT_COLLECTION
    DIGI_CONTACT_COLLECTION DIGI_CONTACT_COLLECTION
#undef DIGI_CONTACT_COLLECTION

    0x05, 0x0D, 0x27, 0xFF, 0xFF, 0x00, 0x00, 0x75, 0x10, 0x95, 0x01,
    0x09, 0x56, 0x81, 0x02,              // Input ScanTime u16 (lmax 65535)
    0x85, 0x0C,                          // Report ID 0x0C (re-set, template)
    0x09, 0x55, 0x75, 0x08, 0x95, 0x04, 0x26, 0xFF, 0x00,
    0xB1, 0x02,                          // Feature Maximum Contact Count 4xB

    // appended for the Linux binding — not part of the template:
    0x06, 0x00, 0xFF, 0x09, 0xC6,        // Page(FF00) Usage(0xC6)
    0x75, 0x08, 0x95, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00,
    0x85, 0x0D, 0xB1, 0x02,              // Feature id 0x0D, 1 byte
    0xC0                                 // End Collection
};
inline constexpr std::size_t kRdescSize = sizeof(kRdesc);

// Range-encoding guard: these asserts derive from the named constants, so a
// hand-typed byte cannot agree with a wrong expectation. Every LogMax/PhysMax
// item must equal its constant, exactly kFrontContacts times, and an earlier
// wrong X value (0x766C = 30316) must be absent.
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

// ---- front identity (Microsoft vendor id, with a product id distinct from
// the physical ports 0x0C46/0x0C5D and from the pad front's 0x4C46) ----
inline constexpr unsigned kVendor = 0x045E;
inline constexpr unsigned kProduct = 0x4D54;
inline constexpr const char* kName = "Surface Digitizer Touch";

// ---- report packing ---------------------------------------------------------
// Input 0x0C, 39 B payload, template field order, LSB-first (HID 6.2.2.2 —
// the same writer discipline as the pad's ptp_pack.hpp; note the digitizer
// order is Tip-then-Confidence, NOT the pad's Confidence-then-Tip):
//   [0] Cnst 0 | [1] ContactCount | 5 x { tip@b0.0, cnst, cfd@b0.2, pad,
//   ContactID u16, X u16, Y u16 } | ScanTime u16.
// ContactCount = the number of Contact ENTRIES the engine reported this
// frame (actives + retire-frame releases, as heatd packs them) — unused
// array slots stay all-zero exactly as the pad packer leaves them.
inline void put_bits(std::uint8_t* p, std::size_t at, std::uint32_t v,
                     unsigned n) {
  for (unsigned i = 0; i < n; ++i) {
    const std::size_t k = at + i;
    const unsigned m = 1u << (k & 7u);
    unsigned byte = p[k >> 3];
    byte = (byte & ~m) | (((v >> i) & 1u) ? m : 0u);
    p[k >> 3] = static_cast<std::uint8_t>(byte);
  }
}

inline void pack_report_0x0C(std::uint8_t* out, std::size_t payload_bytes,
                             std::uint16_t scan_time,
                             const std::vector<Contact>& contacts) {
  for (std::size_t i = 0; i < payload_bytes; ++i) out[i] = 0;
  std::size_t bit = 8;  // [0] = the constant header byte
  put_bits(out, bit, static_cast<std::uint32_t>(contacts.size()), 8);
  bit += 8;
  for (unsigned slot = 0; slot < kFrontContacts; ++slot) {
    const Contact* c = slot < contacts.size() ? &contacts[slot] : nullptr;
    if (!c) { bit += 56; continue; }  // zeroed slot
    put_bits(out, bit + 0, c->in_contact ? 1u : 0u, 1);   // Tip Switch
    put_bits(out, bit + 2, c->confidence ? 1u : 0u, 1);   // Confidence
    put_bits(out, bit + 8, c->slot, 16);                  // Contact ID
    put_bits(out, bit + 24, c->x, 16);                    // X  (0.01 mm)
    put_bits(out, bit + 40, c->y, 16);                    // Y
    bit += 56;
  }
  put_bits(out, bit, scan_time, 16);                      // Scan Time
}

// GET_REPORT answers. The id byte is not optional: uhid.c copies
// UHID_GET_REPORT_REPLY.data verbatim into the raw-request buffer
// (uhid_hid_get_report), and
// mt_get_feature() REJECTS a reply whose first byte is not the report id
// ("Returned feature report did not match the request", hid-multitouch.c
// :548-554) — with the reply dropped, field->value[0] stays 0 and the
// fallback chain needs the declared logical maximum to be
// <= MT_MAX_MAXCONTACT 250 (:243,:577) or it lands on
// MT_DEFAULT_MAXCONTACT 10 (:1428). Our MaxCount declares lmax 255, so a
// value-less fallback is not available: without the id byte the front
// shows ABS_MT_SLOT max=9 instead of 4. (The pad's heatd front replies
// without an id byte; that path is left unchanged.)
// Values: max-contact = kFrontContacts in every MaxCount byte; capability 0
// = present.
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

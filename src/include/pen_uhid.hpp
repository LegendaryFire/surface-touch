// SPDX-License-Identifier: MIT
// The virtual pen: the digitizer's own Pen collection, copied byte for byte.
// The real device declares the same report but sends nothing on it while
// the stream is armed.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "wire.hpp"

namespace digi {
namespace penhid {

inline constexpr std::uint8_t kReportInput = 0x01;
inline constexpr std::size_t kInputPayloadBytes = 15;

inline constexpr std::uint16_t kExtentX = 9600;
inline constexpr std::uint16_t kExtentY = 7200;
inline constexpr std::uint16_t kPhysX = 3031;  // 0.1 mm
inline constexpr std::uint16_t kPhysY = 2010;
inline constexpr std::uint16_t kPressureMax = 4096;
inline constexpr std::uint16_t kTiltMax = 18000;
inline constexpr std::int16_t kTiltPhysMin = -9000;  // 0.01 degree
inline constexpr std::int16_t kTiltPhysMax = 9000;
inline constexpr std::uint16_t kTiltZero = 9000;     // upright
inline constexpr std::uint16_t kScanTimeMax = 65535;

inline constexpr std::uint8_t kRdesc[] = {
    0x05, 0x0D, 0x09, 0x02, 0xA1, 0x01,  // App (Digitizer:Pen)
    0x85, 0x01,                          // Report ID 1
    0x09, 0x20,                          //  Usage (Stylus)
    0x35, 0x00,                          //  Phys Min 0
    0xA1, 0x00,                          //  Collection (Physical)
    0x09, 0x32, 0x09, 0x42, 0x09, 0x44,  //   InRange, TipSwitch,
    0x09, 0x3C, 0x09, 0x45,              //   BarrelSwitch, Invert, Eraser
    0x15, 0x00, 0x25, 0x01,              //   Log 0..1
    0x75, 0x01, 0x95, 0x05,              //   5 × 1 bit
    0x81, 0x02,                          //   Input (Data) — bits 0..4
    0x95, 0x03,                          //   3 × count
    0x81, 0x03,                          //   Input (Cnst) — bits 5..7
    0x05, 0x01,                          //   Usage Page (GD)
    0x09, 0x30,                          //   Usage (X)
    0x75, 0x10, 0x95, 0x01,              //   u16 × 1
    0xA4,                                //   Push
    0x55, 0x0E, 0x65, 0x11,              //   UnitExp −2, Unit cm^1
    0x46, 0xD7, 0x0B,                    //   PhysMax 3031
    0x26, 0x80, 0x25,                    //   LogMax 9600
    0x81, 0x02,                          //   Input X
    0x09, 0x31,                          //   Usage (Y)
    0x46, 0xDA, 0x07,                    //   PhysMax 2010
    0x26, 0x20, 0x1C,                    //   LogMax 7200
    0x81, 0x02,                          //   Input Y
    0xB4,                                //   Pop
    0x05, 0x0D,                          //   Usage Page (Digitizer)
    0x09, 0x30,                          //   Usage (TipPressure)
    0x26, 0x00, 0x10,                    //   LogMax 4096
    0x81, 0x02,                          //   Input TipPressure
    0xA4,                                //   Push
    0x55, 0x0E, 0x65, 0x14,              //   UnitExp −2, Unit degrees
    0x36, 0xD8, 0xDC,                    //   PhysMin −9000
    0x46, 0x28, 0x23,                    //   PhysMax  9000
    0x26, 0x50, 0x46,                    //   LogMax 18000
    0x09, 0x3D, 0x81, 0x02,              //   Input Tilt X
    0x09, 0x3E, 0x81, 0x02,              //   Input Tilt Y
    0x55, 0x0C, 0x66, 0x01, 0x10,        //   UnitExp +12, Unit 0x1001
    0x35, 0x00,                          //   PhysMin 0
    0x47, 0xFF, 0xFF, 0x00, 0x00,        //   PhysMax 65535
    0x27, 0xFF, 0xFF, 0x00, 0x00,        //   LogMax 65535
    0x09, 0x56, 0x81, 0x02,              //   Input ScanTime
    0xB4,                                //   Pop
    0x06, 0x00, 0xFF,                    //   Usage Page (FF00 vendor)
    0x09, 0x01,                          //   Usage 1
    0x75, 0x08, 0x95, 0x01,              //   u8 × 1
    0x81, 0x02,                          //   Input vendor byte
    0x81, 0x03,                          //   Input (Cnst) pad
    0xC0,                                //  End Collection (Physical)
    0xC0                                 // End Collection (App)
};
inline constexpr std::size_t kRdescSize = sizeof(kRdesc);

namespace detail {
constexpr std::uint16_t le16c(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}
constexpr int count_item(const std::uint8_t* d, std::size_t n, std::uint8_t op,
                         std::uint16_t v) {
  int c = 0;
  for (std::size_t i = 0; i + 2 < n; ++i)
    if (d[i] == op && le16c(d + i + 1) == v) ++c;
  return c;
}
constexpr int count_byte(const std::uint8_t* d, std::size_t n, std::uint8_t b) {
  int c = 0;
  for (std::size_t i = 0; i < n; ++i) if (d[i] == b) ++c;
  return c;
}
constexpr int count_pair(const std::uint8_t* d, std::size_t n, std::uint8_t b0,
                         std::uint8_t b1) {
  int c = 0;
  for (std::size_t i = 0; i + 1 < n; ++i)
    if (d[i] == b0 && d[i + 1] == b1) ++c;
  return c;
}
}  // namespace detail
static_assert(detail::count_item(kRdesc, kRdescSize, 0x26, kExtentX) == 1,
              "X logical maximum must be kExtentX");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x46, kPhysX) == 1,
              "X physical maximum must be kPhysX");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x26, kExtentY) == 1,
              "Y logical maximum must be kExtentY");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x46, kPhysY) == 1,
              "Y physical maximum must be kPhysY");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x26, kPressureMax) == 1,
              "TipPressure logical maximum must be kPressureMax");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x26, kTiltMax) == 1,
              "Tilt logical maximum must be kTiltMax (declared once, shared by both tilts)");
static_assert(detail::count_item(kRdesc, kRdescSize, 0x46, std::uint16_t(kTiltPhysMax)) == 1,
              "Tilt physical maximum must be +9000");
static_assert(detail::count_byte(kRdesc, kRdescSize, 0x85) == 1 &&
                  detail::count_pair(kRdesc, kRdescSize, 0x85, kReportInput) == 1,
              "exactly one Report ID item (1-byte operand), id 1");
static_assert(detail::count_pair(kRdesc, kRdescSize, 0x09, 0x51) == 0,
              "NO ContactID usage anywhere — its presence would move the device to "
              "HID_GROUP_MULTITOUCH and hid-multitouch would claim the front");
static_assert(detail::count_byte(kRdesc, kRdescSize, 0xB1) == 0,
              "NO Feature main item anywhere — the collection declares none");

inline constexpr unsigned kVendor = 0x045E;
inline constexpr unsigned kProduct = 0x5045;
inline constexpr const char* kName = "Surface Digitizer Pen";

// Values in the descriptor's logical ranges.
struct PenReport {
  bool in_range = false, tip = false, barrel = false, invert = false, eraser = false;
  std::uint16_t x = 0, y = 0, pressure = 0;
  std::uint16_t tilt_x = kTiltZero, tilt_y = kTiltZero;
  std::uint16_t scan_time = 0;
  std::uint8_t vendor = 0;
};

using wire::put_bits;

inline void pack_report_0x01(std::uint8_t* out, const PenReport& r) {
  for (std::size_t i = 0; i < kInputPayloadBytes; ++i) out[i] = 0;
  put_bits(out, 0, r.in_range ? 1u : 0u, 1);
  put_bits(out, 1, r.tip ? 1u : 0u, 1);
  put_bits(out, 2, r.barrel ? 1u : 0u, 1);
  put_bits(out, 3, r.invert ? 1u : 0u, 1);
  put_bits(out, 4, r.eraser ? 1u : 0u, 1);
  put_bits(out, 8, r.x, 16);
  put_bits(out, 24, r.y, 16);
  put_bits(out, 40, r.pressure, 16);
  put_bits(out, 56, r.tilt_x, 16);
  put_bits(out, 72, r.tilt_y, 16);
  put_bits(out, 88, r.scan_time, 16);
  put_bits(out, 104, r.vendor, 8);
}

// The collection has no feature reports.
inline std::vector<std::uint8_t> feature_reply(std::uint8_t) { return {}; }

}  // namespace penhid
}  // namespace digi

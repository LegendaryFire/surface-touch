// SPDX-License-Identifier: MIT
// The digid pen uhid front: the device's own declared Pen collection
// presented to the host over uhid, so decoded pen input reaches `hid-input`
// under the name `Surface Digitizer Pen`.
//
//   * Grammar and capabilities: the `000D:0002` Application collection of
//     this device's report descriptor (1444 B, bytes 1102..1239), copied
//     item for item. On the physical device this collection's input report
//     `0x01` is declared but sends nothing while the port is armed; this
//     front fills that gap.
//   * The kernel reads the identical collection from the physical device
//     (`quickspi-hid 045E:0C5D Stylus`: BTN_TOOL_PEN/RUBBER, BTN_TOUCH,
//     BTN_STYLUS; ABS_X 0..9600 res 32; ABS_Y 0..7200 res 36; ABS_PRESSURE
//     0..4096; ABS_TILT_X/Y 0..18000 res 5730; ABS_MISC 0..65535;
//     INPUT_PROP_DIRECT), the derived form of the extents below.
//
// GROUP-SAFETY (the reason this reaches hid-input, not hid-multitouch): the
// input report carries NO ContactID (0D:51) and the collection declares NO
// feature — hid_scan_input_usage() never promotes the group, exactly as for
// the physical device whose Stylus node is bound by hid-input today.
//
// Which decoded value feeds which field is pen_feed.hpp's policy. Tilt and
// the barrel/eraser state travel on the pen's Bluetooth link, not this wire,
// and the contact id is not a wire byte. The packer below is field-exact.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "wire.hpp"

namespace digi {
namespace penhid {

// ---- report identity (the device's: Report ID 1, 120-bit payload = 15 B,
// matching the declared Input 0x01 = 16 B with the id byte) ----
inline constexpr std::uint8_t kReportInput = 0x01;
inline constexpr std::size_t kInputPayloadBytes = 15;  // 120 bits, no id byte

// ---- declared extents (device descriptor items, quoted per field) ----
inline constexpr std::uint16_t kExtentX = 9600;        // LogMax; PhysMax 3031 (0.1 mm)
inline constexpr std::uint16_t kExtentY = 7200;        // LogMax; PhysMax 2010 (0.1 mm)
inline constexpr std::uint16_t kPhysX = 3031;
inline constexpr std::uint16_t kPhysY = 2010;
inline constexpr std::uint16_t kPressureMax = 4096;    // TipPressure LogMax; the
                                                       // decoder scales onto it
                                                       // (pen_pressure.hpp).
inline constexpr std::uint16_t kTiltMax = 18000;       // Tilt X/Y LogMax 0..18000
inline constexpr std::int16_t kTiltPhysMin = -9000;    // PhysMin/PhysMax ±9000 = ±90.00°
inline constexpr std::int16_t kTiltPhysMax = 9000;
inline constexpr std::uint16_t kTiltZero = 9000;       // logical for physical 0° (upright)
inline constexpr std::uint16_t kScanTimeMax = 65535;   // usage 0D:56 u16 → kernel ABS_MISC

// ---- the descriptor: device bytes 1102..1239, verbatim, ZERO edits ----
// (App-open; Report ID 1; Usage Stylus 0D:20 Physical child — hid-input maps
// InRange→BTN_TOOL_PEN through it; five 1-bit flag usages in declared ORDER
// InRange, TipSwitch, BarrelSwitch, Invert, Eraser; 3-bit pad; GD X/Y u16
// with unit cm exponent −2 (0.1 mm physical) inside a Push/Pop; Digitizer
// page: TipPressure u16; Tilt X/Y u16 unit degrees exponent −2 inside a
// Push/Pop; usage 56 u16 (ScanTime globals); vendor FF00:0001 u8; final
// 8-bit pad; two End Collections. Bracketed numbers are the device-
// descriptor byte offsets.)
inline constexpr std::uint8_t kRdesc[] = {
    0x05, 0x0D, 0x09, 0x02, 0xA1, 0x01,  // App (Digitizer:Pen)          [1102]
    0x85, 0x01,                          // Report ID 1                  [1108]
    0x09, 0x20,                          //  Usage (Stylus)              [1110]
    0x35, 0x00,                          //  Phys Min 0                  [1112]
    0xA1, 0x00,                          //  Collection (Physical)       [1114]
    0x09, 0x32, 0x09, 0x42, 0x09, 0x44,  //   InRange, TipSwitch,        [1116]
    0x09, 0x3C, 0x09, 0x45,              //   BarrelSwitch, Invert, Eraser
    0x15, 0x00, 0x25, 0x01,              //   Log 0..1                   [1126]
    0x75, 0x01, 0x95, 0x05,              //   5 × 1 bit                  [1130]
    0x81, 0x02,                          //   Input (Data) — bits 0..4   [1134]
    0x95, 0x03,                          //   3 × count                  [1136]
    0x81, 0x03,                          //   Input (Cnst) — bits 5..7   [1138]
    0x05, 0x01,                          //   Usage Page (GD)            [1140]
    0x09, 0x30,                          //   Usage (X)                  [1142]
    0x75, 0x10, 0x95, 0x01,              //   u16 × 1                    [1144]
    0xA4,                                //   Push                       [1148]
    0x55, 0x0E, 0x65, 0x11,              //   UnitExp −2, Unit cm^1      [1149]
    0x46, 0xD7, 0x0B,                    //   PhysMax 3031               [1153]
    0x26, 0x80, 0x25,                    //   LogMax 9600                [1156]
    0x81, 0x02,                          //   Input X                    [1159]
    0x09, 0x31,                          //   Usage (Y)                  [1161]
    0x46, 0xDA, 0x07,                    //   PhysMax 2010               [1163]
    0x26, 0x20, 0x1C,                    //   LogMax 7200                [1166]
    0x81, 0x02,                          //   Input Y                    [1169]
    0xB4,                                //   Pop                        [1171]
    0x05, 0x0D,                          //   Usage Page (Digitizer)     [1172]
    0x09, 0x30,                          //   Usage (TipPressure)        [1174]
    0x26, 0x00, 0x10,                    //   LogMax 4096                [1176]
    0x81, 0x02,                          //   Input TipPressure          [1179]
    0xA4,                                //   Push                       [1181]
    0x55, 0x0E, 0x65, 0x14,              //   UnitExp −2, Unit degrees   [1182]
    0x36, 0xD8, 0xDC,                    //   PhysMin −9000              [1186]
    0x46, 0x28, 0x23,                    //   PhysMax  9000              [1189]
    0x26, 0x50, 0x46,                    //   LogMax 18000               [1192]
    0x09, 0x3D, 0x81, 0x02,              //   Input Tilt X               [1195]
    0x09, 0x3E, 0x81, 0x02,              //   Input Tilt Y               [1199]
    0x55, 0x0C, 0x66, 0x01, 0x10,        //   UnitExp +12, Unit 0x1001   [1203]
    0x35, 0x00,                          //   PhysMin 0                  [1208]
    0x47, 0xFF, 0xFF, 0x00, 0x00,        //   PhysMax 65535              [1210]
    0x27, 0xFF, 0xFF, 0x00, 0x00,        //   LogMax 65535               [1215]
    0x09, 0x56, 0x81, 0x02,              //   Input usage 56 (ScanTime)  [1220]
    0xB4,                                //   Pop                        [1224]
    0x06, 0x00, 0xFF,                    //   Usage Page (FF00 vendor)   [1225]
    0x09, 0x01,                          //   Usage 1                    [1228]
    0x75, 0x08, 0x95, 0x01,              //   u8 × 1                     [1230]
    0x81, 0x02,                          //   Input vendor byte          [1234]
    0x81, 0x03,                          //   Input (Cnst) final pad     [1236]
    0xC0,                                //  End Collection (Physical)   [1238]
    0xC0                                 // End Collection (App)         [1239]
};
inline constexpr std::size_t kRdescSize = sizeof(kRdesc);

// Range-encoding guards: expectations derive from the named constants,
// never from re-typed hex.
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

// ---- front identity (Microsoft vendor, product distinct from
// the physical ports 0x0C46/0x0C5D, the pad front 0x4C46 and the MT front
// 0x4D54; 'PE') ----
inline constexpr unsigned kVendor = 0x045E;
inline constexpr unsigned kProduct = 0x5045;
inline constexpr const char* kName = "Surface Digitizer Pen";

// ---- report packing (wire order of the declared items, LSB-first, HID
// 6.2.2.2 — the same writer discipline as digi_uhid.hpp; local copy so the
// pen front builds standalone):
//   bit0 InRange | bit1 TipSwitch | bit2 BarrelSwitch | bit3 Invert |
//   bit4 Eraser  | bits5..7 pad | X u16 | Y u16 | TipPressure u16 |
//   TiltX u16 | TiltY u16 | ScanTime u16 | vendor u8 | pad u8.
// Values are LOGICAL (the descriptor's own domains: X 0..9600, pressure
// 0..4096, tilt 0..18000 with kTiltZero = physical 0°). Which decoded field
// feeds which slot is pen_feed.hpp's policy.
struct PenReport {
  bool in_range = false, tip = false, barrel = false, invert = false, eraser = false;
  std::uint16_t x = 0, y = 0, pressure = 0;
  std::uint16_t tilt_x = kTiltZero, tilt_y = kTiltZero;
  std::uint16_t scan_time = 0;
  std::uint8_t vendor = 0;  // FF00:0001 — the device's own field, hidden by hid-input
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
  // bits 112..119: the declared constant pad stays zero.
}

// GET_REPORT answers. The collection declares NO feature report, so there is
// nothing to answer — hid-input requests no features for a Pen collection
// (the physical device's Stylus node never issues one).
inline std::vector<std::uint8_t> feature_reply(std::uint8_t) { return {}; }

}  // namespace penhid
}  // namespace digi

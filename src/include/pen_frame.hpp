// SPDX-License-Identifier: MIT
// Decodes the digitizer's pen frames. A frame holds sections of typed
// records. The ones used are the 0x5B axis profiles (pen position) and the
// four records each pen sub-frame is split across (pressure). Tilt and the
// barrel button are not sent.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "wire.hpp"

namespace pen {

using wire::le16;
using wire::le32;

// [3]  u16 body length, counted from offset 3
// [10] sections: u32 length, kind at +5, group at +8; kind 0xFF holds
//      records from +19
// Record: u8 type, u8 group, u16 body length, body. Type 0xFF ends a list.
inline constexpr std::size_t kOffBodyLen = 3;
inline constexpr std::size_t kOffSections = 10;
inline constexpr std::size_t kSecOffKind = 5;
inline constexpr std::size_t kSecOffGroup = 8;
inline constexpr std::size_t kSecOffRecords = 19;
inline constexpr std::uint8_t kKindRecords = 0xFF;
inline constexpr std::size_t kRecordHeader = 4;

inline constexpr unsigned kMaxRecordType = 0x96;
inline constexpr unsigned kMaxRecordsPerList = 0x64;
inline constexpr std::uint8_t kRecordEnd = 0xFF;

struct Record {
  std::uint8_t type;
  std::uint8_t group;
  std::uint16_t len;
  const std::uint8_t* rec;
  const std::uint8_t* body() const { return rec + kRecordHeader; }
};

enum class Fail {
  None, Short, BodyLen, Section, RecordOverrun, RecordType, RecordCount,
  NoEnd,
};

// Calls on_section for each section and on_record for each record.
template <class OnSection, class OnRecord>
inline Fail walk_container(const std::uint8_t* buf, std::size_t n,
                           OnSection&& on_section, OnRecord&& on_record) {
  if (n < kOffSections + kSecOffRecords) return Fail::Short;
  const std::size_t body_end = kOffBodyLen + le16(buf + kOffBodyLen);
  if (body_end > n) return Fail::BodyLen;
  std::size_t sec = kOffSections;
  while (sec < body_end) {
    if (sec + kSecOffRecords > n) return Fail::Section;
    const std::uint32_t sec_len = le32(buf + sec);
    if (sec_len < kSecOffRecords || sec + sec_len > n) return Fail::Section;
    const std::size_t sec_end = sec + sec_len;
    const std::uint8_t kind = buf[sec + kSecOffKind];
    on_section(kind, buf[sec + kSecOffGroup], buf + sec, sec_len);
    if (kind == kKindRecords) {
      std::size_t at = sec + kSecOffRecords;
      unsigned count = 0;
      bool ended = false;
      while (at + kRecordHeader <= sec_end) {
        const Record r{buf[at], buf[at + 1], le16(buf + at + 2), buf + at};
        if (at + kRecordHeader + r.len > sec_end) return Fail::RecordOverrun;
        if (r.type != kRecordEnd && r.type >= kMaxRecordType) return Fail::RecordType;
        if (++count > kMaxRecordsPerList) return Fail::RecordCount;
        on_record(r);
        at += kRecordHeader + r.len;
        if (r.type == kRecordEnd) { ended = true; break; }
      }
      if (!ended || at != sec_end) return Fail::NoEnd;
    }
    sec = sec_end;
  }
  return sec == body_end ? Fail::None : Fail::Section;
}

// The four parts of a sub-frame.
inline constexpr std::uint8_t kRecSubframeHeader = 0x5F;
inline constexpr std::uint8_t kRecSubframeInfo = 0x5A;
inline constexpr std::uint8_t kRecSignalSlots = 0x5C;
inline constexpr std::uint8_t kRecSubframeConfig = 0x62;

inline constexpr std::uint8_t kRecPenState = 0x57;
inline constexpr std::uint8_t kRecAxisProfiles = 0x5B;
inline constexpr std::uint8_t kRecTick = 0x63;
// Only whether these arrived matters.
inline constexpr std::uint8_t kRec58 = 0x58;
inline constexpr std::uint8_t kRec59 = 0x59;
inline constexpr std::uint8_t kRec5D = 0x5D;
inline constexpr std::uint8_t kRec5E = 0x5E;
inline constexpr std::uint8_t kRec80 = 0x80;

inline constexpr std::uint16_t kLenSubframeHeader = 0x10;
inline constexpr std::uint16_t kLenSubframeInfo = 0x94;
inline constexpr std::uint16_t kLenSubframeConfig = 0x10;
inline constexpr std::uint16_t kLenPenState = 0x40;
inline constexpr std::uint16_t kLen59 = 0x40;
inline constexpr std::uint16_t kLenTick = 4;

inline constexpr std::uint32_t kSeenPenState = 0x01;
inline constexpr std::uint32_t kSeen58or80 = 0x02;
inline constexpr std::uint32_t kSeen59 = 0x04;
inline constexpr std::uint32_t kSeenAxisProfiles = 0x10;
inline constexpr std::uint32_t kSeen5D = 0x40;
inline constexpr std::uint32_t kSeen5E = 0x80;

// Sub-frame modes: 6 arrives with the axis profiles (report 0x0C), 7 and 8
// in 0x0B, 9 and 10 in 0x1A scene frames, and 11 carries pressure (0x0D).
inline constexpr std::uint8_t kModeFirst = 6;
inline constexpr std::uint8_t kModeLast = 11;
inline constexpr std::uint8_t kModePressure = 11;

inline bool mode_known(std::uint8_t mode) {
  return mode >= kModeFirst && mode <= kModeLast;
}

// Records that must arrive in the same frame before a sub-frame is used.
inline std::uint32_t required_markers(std::uint8_t mode) {
  switch (mode) {
    case 6: return kSeenPenState | kSeen58or80 | kSeenAxisProfiles | kSeen5E;
    case 7: case 8: return kSeen58or80;
    case 11: return kSeen58or80 | kSeen59 | kSeen5E;
    default: return 0;  // 9, 10
  }
}

// One signal slot: its overall level and nine I/Q frequency bins.
inline constexpr std::size_t kSlotBytes = 48;
inline constexpr int kBins = 9;
struct Slot {
  std::uint32_t level = 0;
  std::array<std::int16_t, kBins> i{};
  std::array<std::int16_t, kBins> q{};
};
inline Slot read_slot(const std::uint8_t* s) {
  Slot o;
  o.level = le32(s + 0x04);
  for (int k = 0; k < kBins; ++k) {
    o.i[k] = std::int16_t(le16(s + 0x08 + 2 * k));
    o.q[k] = std::int16_t(le16(s + 0x1A + 2 * k));
  }
  return o;
}

// 0x5C body: count at +4, key at +5, mode at +9, then two arrays of `count`
// slots from +12.
inline constexpr std::size_t kSlotsArrays = 12;
inline std::uint16_t signal_slots_len(unsigned count) {
  return std::uint16_t(2 * kSlotBytes * count + kSlotsArrays);
}

inline constexpr std::uint8_t kPartHeader = 1 << 0;
inline constexpr std::uint8_t kPartInfo = 1 << 1;
inline constexpr std::uint8_t kPartSlots = 1 << 2;
inline constexpr std::uint8_t kPartConfig = 1 << 3;
inline constexpr std::uint8_t kAllParts =
    kPartHeader | kPartInfo | kPartSlots | kPartConfig;

struct SubFrame {
  std::uint8_t parts = 0;
  std::uint8_t mode = 0, key = 0;
  std::uint32_t timestamp = 0;
  std::vector<Slot> first;
  std::vector<Slot> second;
  bool absolute_pressure = false;
  bool complete() const { return parts == kAllParts; }
};

// 0x5B body: peak cell (col, row), refined cell, then one u32 signal per
// column and per row from +8. A half-length variant is recognised but not
// decoded.
inline constexpr std::size_t kDimCols = 78;
inline constexpr std::size_t kDimRows = 52;
inline constexpr std::size_t kOffProfileCells = 8;

struct Profile {
  std::uint8_t peak_col = 0, peak_row = 0;
  std::uint8_t refined_col = 0, refined_row = 0;  // 0xFF: none
  bool half = false;
  std::vector<std::uint32_t> col;
  std::vector<std::uint32_t> row;
  bool full() const { return !half && !col.empty() && !row.empty(); }
};

inline bool parse_profiles(const Record& r, Profile& out) {
  const std::uint8_t* b = r.body();
  if (r.len == 4 * (kDimCols + kDimRows) + kOffProfileCells) {
    out.peak_col = b[0]; out.peak_row = b[1];
    out.refined_col = b[2]; out.refined_row = b[3];
    out.col.resize(kDimCols);
    out.row.resize(kDimRows);
    const std::uint8_t* cells = b + kOffProfileCells;
    for (std::size_t c = 0; c < kDimCols; ++c) out.col[c] = le32(cells + 4 * c);
    for (std::size_t r2 = 0; r2 < kDimRows; ++r2)
      out.row[r2] = le32(cells + 4 * (kDimCols + r2));
    return true;
  }
  if (r.len == 4 * (kDimCols / 2 + kDimRows / 2) + kOffProfileCells) {
    out.half = true;
    return true;
  }
  return false;
}

struct Frame {
  Fail fail = Fail::None;
  std::uint32_t seen = 0;
  unsigned rejected = 0;
  bool tick = false;
  bool have_pen_state = false;
  std::uint32_t pen_timestamp = 0;
  bool have_profile = false;
  Profile profile;
  std::vector<SubFrame> released;
  bool held_incomplete = false;
};

// Joins each frame's sub-frame parts back together by mode and key.
class Assembler {
 public:
  Frame decode(const std::uint8_t* buf, std::size_t n) {
    Frame f;
    for (auto& s : pending_) s = SubFrame{};
    f.fail = walk_container(
        buf, n, [](std::uint8_t, std::uint8_t, const std::uint8_t*, std::uint32_t) {},
        [&](const Record& r) { accept(f, r); });
    if (f.fail != Fail::None) return f;
    for (;;) {
      SubFrame* next = nullptr;
      for (auto& s : pending_)
        if (s.complete()) { next = &s; break; }
      if (!next) break;
      SubFrame sf = std::move(*next);
      next->parts = 0;
      const std::uint32_t need = required_markers(sf.mode);
      if (!mode_known(sf.mode) || (f.seen & need) != need) {
        f.held_incomplete = true;
        break;
      }
      f.released.push_back(std::move(sf));
    }
    return f;
  }

 private:
  static constexpr unsigned kSlots = 7;

  // Mode 10 comes in two streams: the first key after a mode-6 header is one,
  // any other key the second.
  SubFrame* slot_for(std::uint8_t mode, std::uint8_t key) {
    switch (mode) {
      case 6: expect_mode10_key_ = true; return &pending_[0];
      case 7: return &pending_[1];
      case 9: return &pending_[2];
      case 10:
        if (expect_mode10_key_) {
          mode10_key_ = key;
          expect_mode10_key_ = false;
          return &pending_[3];
        }
        return &pending_[key == mode10_key_ ? 3 : 4];
      case 11: return &pending_[5];
      case 8: return &pending_[6];
      default: return nullptr;
    }
  }

  void accept(Frame& f, const Record& r) {
    const std::uint8_t* b = r.body();
    switch (r.type) {
      case kRecSubframeHeader: {
        if (r.len != kLenSubframeHeader) { ++f.rejected; return; }
        const std::uint8_t mode = b[5], key = b[4];
        SubFrame* s = slot_for(mode, key);
        if (!s) { ++f.rejected; return; }
        s->mode = mode;
        s->key = key;
        s->timestamp = le32(b);
        s->parts |= kPartHeader;
        return;
      }
      case kRecSubframeInfo: {
        if (r.len != kLenSubframeInfo) { ++f.rejected; return; }
        SubFrame* s = slot_for(b[0x91], b[0x92]);
        if (!s) { ++f.rejected; return; }
        s->parts |= kPartInfo;
        return;
      }
      case kRecSignalSlots: {
        const std::uint8_t count = b[4];
        if (r.len != signal_slots_len(count)) { ++f.rejected; return; }
        SubFrame* s = slot_for(b[9], b[5]);
        if (!s) { ++f.rejected; return; }
        const std::uint8_t* a = b + kSlotsArrays;
        const std::uint8_t* c = a + kSlotBytes * count;
        s->first.clear();
        s->second.clear();
        for (unsigned k = 0; k < count; ++k) {
          s->first.push_back(read_slot(a + kSlotBytes * k));
          s->second.push_back(read_slot(c + kSlotBytes * k));
        }
        s->parts |= kPartSlots;
        return;
      }
      case kRecSubframeConfig: {
        if (r.len != kLenSubframeConfig) { ++f.rejected; return; }
        const std::uint8_t index = b[0xC], key = b[0xD], mode = b[0xE];
        if (mode > 0x10 || index > 8) { ++f.rejected; return; }
        SubFrame* s = slot_for(mode, key);
        if (!s) { ++f.rejected; return; }
        s->absolute_pressure = b[0xB] == 1;
        s->parts |= kPartConfig;
        return;
      }
      case kRecPenState:
        if (r.len != kLenPenState) { ++f.rejected; return; }
        f.pen_timestamp = le32(b);
        f.have_pen_state = true;
        f.seen |= kSeenPenState;
        return;
      case kRec58: case kRec80: f.seen |= kSeen58or80; return;
      case kRec59:
        if (r.len != kLen59) { ++f.rejected; return; }
        f.seen |= kSeen59;
        return;
      case kRecAxisProfiles:
        f.seen |= kSeenAxisProfiles;
        if (parse_profiles(r, f.profile)) f.have_profile = true;
        else ++f.rejected;
        return;
      case kRec5D: f.seen |= kSeen5D; return;
      case kRec5E: f.seen |= kSeen5E; return;
      case kRecTick:
        if (r.len != kLenTick) { ++f.rejected; return; }
        f.tick = b[0] == 1;
        return;
      default: return;
    }
  }

  std::array<SubFrame, kSlots> pending_{};
  bool expect_mode10_key_ = false;
  std::uint8_t mode10_key_ = 0;
};

}  // namespace pen

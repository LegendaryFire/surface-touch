// SPDX-License-Identifier: MIT
// The pen-family replay tally shape, defined once here (same approach as
// digi_replay.hpp).
//
// What is counted:
//   * out_grammar counts ONLY the 16-byte 0x2E frame, which is outside the
//     container grammar;
//   * mode_fit pairs report id to sub-frame mode (0x0C=6, 0x0D=11,
//     0x0B=7|8);
//   * cycle counts steps between RELEASED sub-frames only, the 6→7→11→8
//     cycle;
//   * scene_pen / scene_records are the 0x1A pen-chain class: container
//     kind-0xFF sections holding pen records (modes 9/10); the 0x1A matrix
//     class is not pen and is not counted here;
//   * the position tallies (pos) run pen_position.hpp on every 0x0C frame
//     with a full profile: none = an axis without signal, agree = both axes
//     within ±1 of the controller's refined cell.
#pragma once

#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>

#include "digi_frame.hpp"
#include "pen_frame.hpp"
#include "pen_position.hpp"

namespace pen {

// id -> mode relation and the cycle order.
inline bool mode_fits(std::uint8_t rid, std::uint8_t mode) {
  switch (rid) {
    case 0x0C: return mode == 6;
    case 0x0D: return mode == 11;
    case 0x0B: return mode == 7 || mode == 8;
    default: return false;
  }
}
inline int next_mode(int m) {  // exact cycle 6→7→11→8
  switch (m) {
    case 6: return 7; case 7: return 11; case 11: return 8; case 8: return 6;
  }
  return -1;
}

struct PositionTally {
  long n = 0, none = 0, agree = 0;
};

struct Stats {
  long frames = 0, walk_ok = 0, out_grammar = 0, walk_fail = 0;
  long pen_reports = 0, one_record = 0, mode_fit = 0;
  long cycle_steps = 0, cycle_ok = 0;
  long scene_pen = 0, scene_records = 0;
  long ticks = 0, rejected = 0, held_incomplete = 0;
  std::map<unsigned, long> released;  // sub-frame mode -> count
  PositionTally pos;

  void merge(const Stats& o) {
    frames += o.frames; walk_ok += o.walk_ok; out_grammar += o.out_grammar;
    walk_fail += o.walk_fail; pen_reports += o.pen_reports;
    one_record += o.one_record; mode_fit += o.mode_fit;
    cycle_steps += o.cycle_steps; cycle_ok += o.cycle_ok;
    scene_pen += o.scene_pen; scene_records += o.scene_records;
    ticks += o.ticks; rejected += o.rejected; held_incomplete += o.held_incomplete;
    for (const auto& [m, c] : o.released) released[m] += c;
    pos.n += o.pos.n; pos.none += o.pos.none; pos.agree += o.pos.agree;
  }
};

// Replay one capture file through the shipped decode. Returns frames == 0
// iff the file is unreadable (same convention as digi::for_each_record).
// The optional sink sees every frame before aggregation — measurement only:
// it changes nothing the Stats count.
inline Stats replay_file(
    const char* path,
    const std::function<void(std::uint64_t, const std::uint8_t*, std::uint32_t,
                             const Frame&)>* sink = nullptr) {
  Stats st;
  Assembler as;
  int prev_mode = -1;
  digi::for_each_record(
      path, [&](std::uint64_t us, const std::uint8_t* b, std::uint32_t n) {
        ++st.frames;
        const Frame f = as.decode(b, n);
        if (sink) (*sink)(us, b, n, f);
        if (b[0] == 0x2E && n == 16) { ++st.out_grammar; return; }
        if (f.fail != Fail::None) { ++st.walk_fail; return; }
        ++st.walk_ok;
        st.rejected += f.rejected;
        st.ticks += f.tick;
        st.held_incomplete += f.held_incomplete;
        for (const auto& sf : f.released) ++st.released[sf.mode];
        const std::uint8_t rid = b[0];
        if (rid == 0x0B || rid == 0x0C || rid == 0x0D) {
          ++st.pen_reports;
          if (f.released.size() == 1 && !f.held_incomplete) {
            ++st.one_record;
            const unsigned m = f.released[0].mode;
            st.mode_fit += mode_fits(rid, static_cast<std::uint8_t>(m));
            if (prev_mode >= 0) {
              ++st.cycle_steps;
              st.cycle_ok += next_mode(prev_mode) == static_cast<int>(m);
            }
            prev_mode = static_cast<int>(m);
          }
        } else if (rid == 0x1A &&
                   b[kOffSections + kSecOffKind] == kKindRecords) {
          ++st.scene_pen;
          st.scene_records += static_cast<long>(f.released.size());
        }
        if (rid == 0x0C && f.have_profile && f.profile.full()) {
          ++st.pos.n;
          const PenPosition p = locate_pen(f.profile);
          if (!p.valid()) ++st.pos.none;
          else st.pos.agree += agrees_with_refined_cell(p, f.profile);
        }
      });
  return st;
}

}  // namespace pen

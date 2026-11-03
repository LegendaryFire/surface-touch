// SPDX-License-Identifier: MIT
// The replay/live tally contract, shared by digid's loops so the release
// policy has one definition.
//
// What is counted:
//   * the live-track histogram counts ACTIVES only, taken BEFORE the
//     release-append — so "N live tracks" is not what the front's next
//     ContactCount will be;
//   * releases are counted at drain, but only kFrontContacts-worth ride the
//     report (as in heatd; overflow drops the release and the kernel's
//     sticky timer catches it — the counts still show it);
//   * whole-frame rejection only: a container that fails any anchor check
//     parses to nothing, it never contributes partially.
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "digi_pipeline.hpp"
#include "digi_uhid.hpp"

namespace digi {

struct Stats {
  long records = 0, containers = 0, matrix = 0, scene = 0, other_id = 0;
  long parse_fail = 0, releases = 0;
  std::map<unsigned, unsigned> live_hist;  // concurrent tracks -> frames
};

// As in heatd: releases ride the same report as the actives, capped by the
// array size.
inline void append_releases(DigiEngine& eng, std::vector<Contact>& contacts,
                            long& release_count) {
  for (auto& r : eng.drain_releases()) {
    ++release_count;
    if (contacts.size() < static_cast<std::size_t>(uhid::kFrontContacts))
      contacts.push_back(r);
  }
}

}  // namespace digi

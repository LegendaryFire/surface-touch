// SPDX-License-Identifier: MIT
// digid — Surface digitizer processor.
//
// Usage:
//   sudo ./digid --hidraw /dev/surface-thc-digitizer [--verbose] [--record FILE.binl]
//     GET 0x05 -> SET [05 01] -> stream 0x1A containers into the engine and
//     the uhid MT front ('Surface Digitizer Touch'), pen-family frames into the
//     pen feed and the uhid pen front ('Surface Digitizer Pen') -> SET [05 00]
//     at teardown (Ctrl-C). The fronts are host-side virtual devices.
//     --record writes every report read to FILE.binl, for a bug report.
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <getopt.h>
#include <linux/hidraw.h>
#include <map>
#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <unistd.h>

#include "digi_frame.hpp"
#include "digi_pipeline.hpp"
#include "digi_uhid.hpp"
#include "hidraw_node.hpp"
#include "pen_feed.hpp"
#include "uhid_device.hpp"

namespace {

volatile sig_atomic_t g_stop = 0;
void on_sigint(int) { g_stop = 1; }

// Session counters, printed at teardown. The live-track histogram counts
// active tracks before the release-append.
struct Stats {
  long records = 0, containers = 0, matrix = 0, scene = 0;
  long parse_fail = 0, releases = 0;
  std::map<unsigned, unsigned> live_hist;  // concurrent tracks -> frames
};

// As in heatd: releases ride the same report as the actives, capped by the
// array size.
void append_releases(digi::DigiEngine& eng, std::vector<digi::Contact>& contacts,
                     long& release_count) {
  for (auto& r : eng.drain_releases()) {
    ++release_count;
    if (contacts.size() < static_cast<std::size_t>(digi::uhid::kFrontContacts))
      contacts.push_back(r);
  }
}

// ---- uhid fronts (host-side virtual devices) --------------------------------
// The digitizer's virtual multitouch device and its virtual pen device. Name,
// id and phys are all distinct, so the fronts never collide. The pen
// collection declares no feature report, so its reply is always empty.
uhid::DeviceSpec touch_spec() {
  return {digi::uhid::kName, digi::uhid::kVendor, digi::uhid::kProduct,
          digi::uhid::kRdesc, digi::uhid::kRdescSize, &digi::uhid::feature_reply};
}

uhid::DeviceSpec pen_spec() {
  return {digi::penhid::kName, digi::penhid::kVendor, digi::penhid::kProduct,
          digi::penhid::kRdesc, digi::penhid::kRdescSize, &digi::penhid::feature_reply};
}

// Pen-family wire ids: the containers the pen Assembler walks. 0x1A is the
// touch container (its pen-chain sections are counted, not needed by the pen
// feed), 0x2E is the out-of-grammar 16 B frame.
inline bool is_pen_frame(std::uint8_t id) {
  return id == 0x07 || id == 0x0B || id == 0x0C || id == 0x0D;
}

// Pen-in-range touch suppression: while the pen is in range, and for a short
// hold after it leaves, every touch is a palm - the writing hand resting on
// the screen. The pen may report its last in-range frame and then fall
// silent, so the hold runs from the last in-range report.
class PenProximity {
 public:
  void update(bool in_range) {
    if (in_range) last_in_range_ = Clock::now();
  }
  bool near() const { return Clock::now() - last_in_range_ < kHold; }

 private:
  using Clock = std::chrono::steady_clock;
  static constexpr auto kHold = std::chrono::milliseconds(500);
  Clock::time_point last_in_range_ = Clock::now() - kHold;
};

// ---- live mode ---------------------------------------------------------------
// One fd for the whole session:
//   open node -> check identity -> GET 0x05 -> SET [05 01] -> stream -> SET [05 00] -> close
// The mode belongs to this open, so this process reverts it at teardown.
// GET 0x05 readbacks are logged only; nothing branches on them. The first
// 0x1A container is the signal that the arm took effect. A failed feature
// write is never retried.
inline constexpr std::uint8_t kArmReport = 0x05;  // the only feature id sent
inline constexpr hidraw::Identity kDigitizer{0x045E, 0x0C5D};

int feature_get(int fd, std::uint8_t& mode, const char* when) {
  std::uint8_t buf[2] = {kArmReport, 0};
  if (::ioctl(fd, HIDIOCGFEATURE(2), buf) < 0) {
    std::fprintf(stderr, "digid: GET 0x05 (%s): %s — not retrying; if the "
                 "port is wedged, reload intel_quickspi or reboot\n", when,
                 std::strerror(errno));
    return -1;
  }
  mode = buf[1];
  return 0;
}

int feature_set(int fd, std::uint8_t val, const char* when) {
  std::uint8_t buf[2] = {kArmReport, val};
  if (::ioctl(fd, HIDIOCSFEATURE(2), buf) < 0) {
    std::fprintf(stderr, "digid: SET [05 %02X] (%s): %s — not retrying; if "
                 "the port is wedged, reload intel_quickspi or reboot\n",
                 val, when, std::strerror(errno));
    return -1;
  }
  return 0;
}

// Appends one .binl record: u64 CLOCK_MONOTONIC us, u32 length, payload.
void write_record(std::FILE* f, const std::uint8_t* buf, std::uint32_t len) {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const std::uint64_t us = std::uint64_t(ts.tv_sec) * 1000000u +
                           std::uint64_t(ts.tv_nsec) / 1000u;
  std::uint8_t hdr[12];
  for (int i = 0; i < 8; ++i) hdr[i] = std::uint8_t(us >> (8 * i));
  for (int i = 0; i < 4; ++i) hdr[8 + i] = std::uint8_t(len >> (8 * i));
  std::fwrite(hdr, 1, sizeof(hdr), f);
  std::fwrite(buf, 1, len, f);
}

int live(const char* node, bool verbose, const char* record_path) {
  int fd = ::open(node, O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    std::fprintf(stderr, "digid: open %s: %s (needs root)\n", node, std::strerror(errno));
    return 2;
  }
  // Only the digitizer's transport is ever armed.
  const auto id = hidraw::identity(fd);
  if (!id || id->vendor != kDigitizer.vendor || id->product != kDigitizer.product) {
    std::fprintf(stderr, "digid: %s is not the digitizer (%04x:%04x)\n", node,
                 kDigitizer.vendor, kDigitizer.product);
    ::close(fd);
    return 2;
  }
  const std::string function = hidraw::pci_function(node);
  const std::string phys = function.empty() ? "" : "pci-" + function + "/digid";
  // Open the recording before arming, so a bad path never arms the port.
  std::FILE* rec = nullptr;
  if (record_path) {
    rec = std::fopen(record_path, "wb");
    if (!rec) {
      std::fprintf(stderr, "digid: open %s: %s\n", record_path, std::strerror(errno));
      ::close(fd);
      return 2;
    }
    std::printf("digid: live: recording to %s\n", record_path);
  }
  std::printf("digid: live: node %s (pci %s, front phys %s)\n", node,
              function.c_str(), phys.c_str());
  std::uint8_t mode = 0xFF;
  if (feature_get(fd, mode, "pre-state") == 0)
    std::printf("  GET 0x05 pre-state: mode=%u (a GET readback lags one "
                "transaction — logged, never branched on)\n", mode);
  if (feature_set(fd, 1, "arm") != 0) { ::close(fd); return 4; }
  std::puts("  SET [05 01] answered — mode claimed for this open");
  if (feature_get(fd, mode, "post-arm") == 0)
    std::printf("  GET 0x05 post-arm: mode=%u — 0 here is the expected lag; "
                "the first 0x1A is the commit signal\n", mode);

  uhid::Device front, pfront;
  if (!front.open(touch_spec(), phys)) {  // revert the mode before exiting
    feature_set(fd, 0, "post-open-failure revert");
    ::close(fd);
    return 2;
  }
  const std::string pen_phys = phys + "-pen";  // distinct phys, no collision
  if (!pfront.open(pen_spec(), pen_phys)) {
    feature_set(fd, 0, "post-pen-open-failure revert");
    ::close(fd);
    return 2;
  }
  for (int i = 0; i < 200 && !(front.started() && pfront.started());
       ++i) {
    front.pump();
    pfront.pump();
    usleep(10000);
  }
  std::printf("digid: live: fronts '%s' + '%s' up; stream is the truth — "
              "Ctrl-C reverts and exits\n", digi::uhid::kName,
              digi::penhid::kName);

  Stats st;
  std::map<unsigned, unsigned> id_hist;
  digi::DigiEngine eng;
  pen::Assembler pasm;
  pen::feed::PenFeed pfeed;
  PenProximity pen_proximity;
  std::uint8_t buf[8192];  // larger than every declared report (max 7488 B)
  long sent = 0, pen_frames = 0, pen_sent = 0;
  bool first_frame_logged = false;
  while (!g_stop) {
    const ssize_t n = ::read(fd, buf, sizeof(buf));
    if (n > 0) {
      ++st.records;
      ++id_hist[buf[0]];
      if (rec) write_record(rec, buf, static_cast<std::uint32_t>(n));
      if (n == static_cast<ssize_t>(digi::kWireSize) && buf[0] == digi::kReportId) {
        ++st.containers;
        if (!first_frame_logged) {
          first_frame_logged = true;
          std::puts("  first 0x1A container — the arm committed");
        }
        digi::Fail why = digi::Fail::None;
        const auto f = digi::parse_container(buf, static_cast<std::uint32_t>(n), &why);
        if (!f) {
          ++st.parse_fail;
        } else if (!f->matrix) {
          ++st.scene;
        } else {
          ++st.matrix;
          auto contacts = eng.process(f->cells, pen_proximity.near());
          ++st.live_hist[contacts.size()];
          append_releases(eng, contacts, st.releases);
          std::uint8_t payload[digi::uhid::kInputPayloadBytes];
          digi::uhid::pack_report_0x0C(payload, sizeof(payload), f->ts16, contacts);
          if (front.send_input(digi::uhid::kReportInput, payload, sizeof(payload)))
            ++sent;
          if (verbose)
            std::printf("matrix ts16=%5u tracks=%zu sent=%ld\n", f->ts16,
                        contacts.size(), sent);
        }
      } else if (n >= 29 && is_pen_frame(buf[0])) {
        // Pen family: walk + assemble, judge the pen feed per full-profile
        // frame, send one 15 B Report ID 1 into the pen front.
        ++pen_frames;
        const auto pf = pasm.decode(buf, static_cast<std::uint32_t>(n));
        if (pf.fail != pen::Fail::None) {
          ++st.parse_fail;  // same counter: a frame that will not walk
        } else if (pfeed.update(pf)) {
          pen_proximity.update(pfeed.report.in_range);
          std::uint8_t ppayload[digi::penhid::kInputPayloadBytes];
          digi::penhid::pack_report_0x01(ppayload, pfeed.report);
          if (pfront.send_input(digi::penhid::kReportInput, ppayload,
                                sizeof(ppayload)))
            ++pen_sent;
          if (verbose)
            std::printf("pen in=%d tip=%d x=%5u y=%5u scan=%5u sent=%ld\n",
                        pfeed.report.in_range, pfeed.report.tip,
                        pfeed.report.x, pfeed.report.y, pfeed.report.scan_time,
                        pen_sent);
        }
      }
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      std::fprintf(stderr, "digid: hidraw read: %s\n", std::strerror(errno));
      break;
    }
    front.pump();
    pfront.pump();
    usleep(2000);  // 2 ms poll; also keeps the uhid pump serviced when idle
  }

  // Teardown: revert the mode. feature_set prints the recovery on failure.
  if (feature_set(fd, 0, "teardown revert") == 0)
    std::puts("  SET [05 00] answered — mode released at teardown");
  if (feature_get(fd, mode, "post-revert") == 0)
    std::printf("  GET 0x05 post-revert: mode=%u — 1 here is the expected "
                "lag\n", mode);
  ::close(fd);
  if (rec) std::fclose(rec);
  // The front is destroyed after this (UHID_DESTROY), after the stream.
  std::printf("digid: live done: records=%ld 0x1A=%ld matrix=%ld scene=%ld "
              "parse_fail=%ld releases=%ld mt-reports-sent=%ld "
              "pen-frames=%ld pen-reports-sent=%ld\n",
              st.records, st.containers, st.matrix, st.scene, st.parse_fail,
              st.releases, sent, pen_frames, pen_sent);
  std::puts("  id histogram (the pen family is decoded by the pen path "
            "above; every other non-0x1A id is counted, not decoded):");
  for (const auto& [id, n] : id_hist)
    std::printf("    id 0x%02X: %u\n", id, n);
  for (const auto& [live_n, frames] : st.live_hist)
    std::printf("    matrix frames with %u live track(s): %u\n", live_n, frames);
  return 0;
}

}  // namespace

void usage(std::FILE* out) {
  std::fprintf(out,
               "usage: digid --hidraw NODE [--verbose] [--record FILE.binl]\n"
               "  NODE is the digitizer's hidraw node (/dev/surface-thc-digitizer).\n"
               "  Arms the digitizer port (SET_FEATURE [05 01]), serves the\n"
               "  'Surface Digitizer Touch' and 'Surface Digitizer Pen' fronts, and\n"
               "  reverts the port at teardown. --record saves every report\n"
               "  read, for a bug report.\n");
}

int main(int argc, char** argv) {
  const char* node = nullptr;
  const char* record_path = nullptr;
  bool verbose = false;
  enum : int { kHidraw = 256, kRecord };
  static const option kOptions[] = {
      {"hidraw", required_argument, nullptr, kHidraw},
      {"record", required_argument, nullptr, kRecord},
      {"verbose", no_argument, nullptr, 'v'},
      {"help", no_argument, nullptr, 'h'},
      {},
  };
  for (int c; (c = getopt_long(argc, argv, "vh", kOptions, nullptr)) != -1;) {
    switch (c) {
      case kHidraw: node = optarg; break;
      case kRecord: record_path = optarg; break;
      case 'v': verbose = true; break;
      case 'h': usage(stdout); return 0;
      default: usage(stderr); return 2;  // getopt_long has named the problem
    }
  }
  if (optind < argc) std::fprintf(stderr, "digid: unexpected argument %s\n", argv[optind]);
  if (!node || optind < argc) {
    usage(stderr);
    return 2;
  }
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  // SIGHUP too: the teardown revert is the only thing that releases the arm.
  // Closing the node does not: a digid killed by SIGHUP without a handler
  // left the port armed, with the firmware stream off and no client alive.
  std::signal(SIGHUP, on_sigint);
  // SIGUSR1 has no use here; ignore it rather than die with the port armed.
  std::signal(SIGUSR1, SIG_IGN);
  return live(node, verbose, record_path);
}

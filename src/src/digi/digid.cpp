// SPDX-License-Identifier: MIT
// digid — Surface digitizer processor.
//
// Usage:
//   sudo ./digid --live [--bdf 0000:00:10.6] [--verbose] [--record FILE.binl]
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
#include <dirent.h>
#include <fcntl.h>
#include <libgen.h>  // POSIX basename (driver-name extraction)
#include <limits.h>
#include <linux/hidraw.h>
#include <linux/uhid.h>
#include <map>
#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <unistd.h>

#include "digi_frame.hpp"
#include "digi_pipeline.hpp"
#include "digi_uhid.hpp"
#include "pen_feed.hpp"

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

// ---- uhid front (host-side virtual device) ----------------------------------
// One class, two instances: the digitizer's virtual multitouch device and
// the virtual pen device. They differ ONLY in the FrontSpec below: name, id
// and phys are all distinct, so the fronts never collide (the phys arrives
// per front from the caller). CREATE2 with the spec's descriptor, INPUT2 per
// judged frame, and replies to the kernel's feature requests (the pen
// collection declares no feature report, so its reply is the empty vector).
struct FrontSpec {
  const char* name;
  unsigned vendor, product;
  const std::uint8_t* rd;
  std::size_t rd_size;
  std::vector<std::uint8_t> (*reply)(std::uint8_t);
};

constexpr FrontSpec kTouchSpec{digi::uhid::kName,   digi::uhid::kVendor,
                               digi::uhid::kProduct, digi::uhid::kRdesc,
                               digi::uhid::kRdescSize,
                               &digi::uhid::feature_reply};
constexpr FrontSpec kPenSpec{digi::penhid::kName,    digi::penhid::kVendor,
                             digi::penhid::kProduct, digi::penhid::kRdesc,
                             digi::penhid::kRdescSize,
                             &digi::penhid::feature_reply};

class UhidFront {
 public:
  // phys is copied into the HID device's phys string ("pci-<bdf>/digid"
  // for the touch front, "pci-<bdf>/digid-pen" for the pen front).
  bool open_front(const char* phys, const FrontSpec& spec = kTouchSpec) {
    spec_ = &spec;
    fd_ = ::open("/dev/uhid", O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      std::fprintf(stderr, "digid: open /dev/uhid: %s\n", std::strerror(errno));
      return false;
    }
    uhid_event ev{};
    ev.type = UHID_CREATE2;
    auto& c = ev.u.create2;
    std::snprintf(reinterpret_cast<char*>(c.name), sizeof(c.name), "%s",
                  spec_->name);
    std::snprintf(reinterpret_cast<char*>(c.phys), sizeof(c.phys), "%s", phys);
    c.rd_size = static_cast<__u16>(spec_->rd_size);
    c.bus = 0x18;  // BUS_VIRTUAL
    c.vendor = spec_->vendor;
    c.product = spec_->product;
    c.version = 0x0001;
    c.country = 0;
    std::memcpy(c.rd_data, spec_->rd, spec_->rd_size);
    const std::size_t bytes =
        offsetof(uhid_event, u.create2.rd_data) + spec_->rd_size;
    if (::write(fd_, &ev, bytes) != static_cast<ssize_t>(bytes)) {
      std::fprintf(stderr, "digid: UHID_CREATE2: %s\n", std::strerror(errno));
      return false;
    }
    created_ = true;
    return true;
  }
  ~UhidFront() {
    if (fd_ >= 0) {
      if (created_) {
        uhid_event ev{};
        ev.type = UHID_DESTROY;
        if (::write(fd_, &ev, sizeof(ev)) < 0) std::fprintf(stderr, "digid: UHID_DESTROY failed\n");
      }
      ::close(fd_);
    }
  }
  bool send_input(std::uint8_t report_id, const std::uint8_t* payload,
                  std::size_t n) {
    uhid_event ev{};
    ev.type = UHID_INPUT2;
    ev.u.input2.size = static_cast<__u16>(n + 1);
    ev.u.input2.data[0] = report_id;
    std::memcpy(ev.u.input2.data + 1, payload, n);
    const std::size_t bytes = offsetof(uhid_event, u.input2.data) + n + 1;
    return ::write(fd_, &ev, bytes) == static_cast<ssize_t>(bytes);
  }
  // Handle pending kernel events without blocking.
  void pump() {
    for (;;) {
      uhid_event ev;
      const ssize_t r = ::read(fd_, &ev, sizeof(ev));
      if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
      if (r != static_cast<ssize_t>(sizeof(ev))) return;
      switch (ev.type) {
        case UHID_START: started_ = true; break;
        case UHID_STOP: started_ = false; break;
        case UHID_GET_REPORT: {
          auto v = spec_->reply(static_cast<std::uint8_t>(ev.u.get_report.rnum));
          uhid_event rep{};
          rep.type = UHID_GET_REPORT_REPLY;
          rep.u.get_report_reply.id = ev.u.get_report.id;
          rep.u.get_report_reply.err = 0;
          rep.u.get_report_reply.size = static_cast<__u16>(v.size());
          if (!v.empty())
            std::memcpy(rep.u.get_report_reply.data, v.data(), v.size());
          const std::size_t bytes =
              offsetof(uhid_event, u.get_report_reply.data) + v.size();
          if (::write(fd_, &rep, bytes) != static_cast<ssize_t>(bytes)) return;
          break;
        }
        case UHID_SET_REPORT: {
          uhid_event rep{};  // the kernel waits for a reply
          rep.type = UHID_SET_REPORT_REPLY;
          rep.u.set_report_reply.id = ev.u.set_report.id;
          if (::write(fd_, &rep, sizeof(rep)) < 0) return;
          break;
        }
        default: break;
      }
    }
  }
  bool started() const { return started_; }
  const char* name() const { return spec_->name; }

 private:
  const FrontSpec* spec_ = &kTouchSpec;
  int fd_ = -1;
  bool created_ = false;
  bool started_ = false;
};

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
//   resolve node -> GET 0x05 -> SET [05 01] -> stream -> SET [05 00] -> close
// The mode belongs to this open, so this process reverts it at teardown.
// GET 0x05 readbacks are logged only; nothing branches on them. The first
// 0x1A container is the signal that the arm took effect. A failed feature
// write is never retried.
inline constexpr std::uint8_t kArmReport = 0x05;  // the only feature id sent
constexpr char kBdfDefault[] = "0000:00:10.6";

// Finds the hidraw node under bdf. Requires bdf to be bound to intel_quickspi
// and refuses the pad function.
bool resolve_hidraw(const char* bdf, std::string& node, std::string& phys) {
  if (!std::strcmp(bdf, "0000:00:10.7")) {
    std::fprintf(stderr, "digid: %s is the PAD function — heatd serves it; "
                 "--live is the digitizer port only\n", bdf);
    return false;
  }
  DIR* dir = ::opendir("/sys/class/hidraw");
  if (!dir) { std::perror("digid: opendir /sys/class/hidraw"); return false; }
  std::string tag = std::string("/") + bdf + "/";
  for (dirent* e; (e = ::readdir(dir)) != nullptr;) {
    if (std::strncmp(e->d_name, "hidraw", 6) != 0) continue;
    char path[PATH_MAX], real[PATH_MAX];
    std::snprintf(path, sizeof(path), "/sys/class/hidraw/%s/device", e->d_name);
    if (!::realpath(path, real)) continue;
    if (!std::strstr(real, tag.c_str())) continue;
    std::snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/driver", bdf);
    char link[PATH_MAX];
    ssize_t n = ::readlink(path, link, sizeof(link) - 1);
    if (n <= 0) { std::fprintf(stderr, "digid: %s is driver-unclaimed — is "
        "intel_quickspi (surface-thc-hid) installed and loaded?\n", bdf);
      ::closedir(dir); return false; }
    link[n] = '\0';
    if (std::strcmp(::basename(link), "intel_quickspi") != 0) {
      std::fprintf(stderr, "digid: %s is bound to '%s', not intel_quickspi\n",
          bdf, ::basename(link));
      ::closedir(dir); return false;
    }
    node = std::string("/dev/") + e->d_name;
    phys = std::string("pci-") + bdf + "/digid";
    ::closedir(dir);
    return true;
  }
  ::closedir(dir);
  std::fprintf(stderr, "digid: no hidraw node under %s (digitizer not bound?)\n", bdf);
  return false;
}

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

int live(const char* bdf, bool verbose, const char* record_path) {
  std::string node, phys;
  if (!resolve_hidraw(bdf, node, phys)) return 2;
  // Open the recording before arming, so a bad path never arms the port.
  std::FILE* rec = nullptr;
  if (record_path) {
    rec = std::fopen(record_path, "wb");
    if (!rec) {
      std::fprintf(stderr, "digid: open %s: %s\n", record_path, std::strerror(errno));
      return 2;
    }
    std::printf("digid: live: recording to %s\n", record_path);
  }
  std::printf("digid: live: node %s (bdf %s, front phys %s)\n",
              node.c_str(), bdf, phys.c_str());
  int fd = ::open(node.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    std::fflush(stdout);  // keep the node line ahead of the error
    std::fprintf(stderr, "digid: open %s: %s (needs root)\n", node.c_str(),
                 std::strerror(errno));
    return 2;
  }
  std::uint8_t mode = 0xFF;
  if (feature_get(fd, mode, "pre-state") == 0)
    std::printf("  GET 0x05 pre-state: mode=%u (a GET readback lags one "
                "transaction — logged, never branched on)\n", mode);
  if (feature_set(fd, 1, "arm") != 0) { ::close(fd); return 4; }
  std::puts("  SET [05 01] answered — mode claimed for this open");
  if (feature_get(fd, mode, "post-arm") == 0)
    std::printf("  GET 0x05 post-arm: mode=%u — 0 here is the expected lag; "
                "the first 0x1A is the commit signal\n", mode);

  UhidFront front, pfront;
  if (!front.open_front(phys.c_str())) {  // revert the mode before exiting
    feature_set(fd, 0, "post-open-failure revert");
    ::close(fd);
    return 2;
  }
  const std::string pen_phys = phys + "-pen";  // distinct phys, no collision
  if (!pfront.open_front(pen_phys.c_str(), kPenSpec)) {
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

int main(int argc, char** argv) {
  const char* bdf = kBdfDefault;
  const char* record_path = nullptr;
  bool verbose = false, live_mode = false;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--live")) live_mode = true;
    else if (!std::strcmp(argv[i], "--bdf") && i + 1 < argc) bdf = argv[++i];
    else if (!std::strcmp(argv[i], "--record") && i + 1 < argc) record_path = argv[++i];
    else if (!std::strcmp(argv[i], "--verbose") || !std::strcmp(argv[i], "-v"))
      verbose = true;
    else {
      live_mode = false;
      break;
    }
  }
  if (!live_mode) {
    std::fprintf(stderr,
                 "usage: digid --live [--bdf BDF] [--verbose] [--record FILE.binl]\n"
                 "  arms the digitizer port (SET_FEATURE [05 01]), serves the\n"
                 "  'Surface Digitizer Touch' and 'Surface Digitizer Pen' fronts, and\n"
                 "  reverts the port at teardown. --record saves every report\n"
                 "  read, for a bug report.\n");
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
  return live(bdf, verbose, record_path);
}

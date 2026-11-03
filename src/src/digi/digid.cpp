// SPDX-License-Identifier: MIT
// digid — Surface digitizer processor.
//
// --replay and --uhid-replay read captures only and never open the device;
// the uhid front is a host-side virtual device. --live arms the port with
// SET_FEATURE [05 01] and reverts it at teardown.
//
// Usage:
//   ./digid --replay FILE.binl [--verbose]
//   ./digid --trace-pen FILE.binl
//     per-frame pen CSV trace: the normal decode with a measurement sink;
//     prints only.
//     tallies + live-track histogram; --verbose per-frame track positions.
//   ./digid --uhid-replay FILE.binl [--gate] [--rate HZ]
//     feed the capture to the uhid fronts (MT 'Surface Digitizer Touch' for the
//     0x1A containers, pen 'Surface Digitizer Pen' for the pen-family frames)
//     at capture cadence (or capped by --rate: a replay is a
//     real event source for the desktop); --gate waits for SIGUSR1 before
//     frame 1. Stays alive after the feed for inspection; Ctrl-C exits and
//     destroys the fronts.
//   sudo ./digid --live [--bdf 0000:00:10.6] [--verbose] [--record FILE.binl]
//     GET 0x05 -> SET [05 01] -> stream 0x1A containers into the engine and
//     the uhid MT front, pen-family frames into the pen feed and the uhid
//     pen front -> SET [05 00] at teardown (Ctrl-C). --record writes
//     every report read to FILE.binl, replayable with --replay.
#include <cerrno>
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
#include <sys/ioctl.h>
#include <unistd.h>

#include "digi_frame.hpp"
#include "digi_pipeline.hpp"
#include "digi_replay.hpp"
#include "digi_uhid.hpp"
#include "pen_feed.hpp"
#include "pen_replay.hpp"

namespace {

using digi::append_releases;
using digi::Stats;

volatile sig_atomic_t g_stop = 0;
volatile sig_atomic_t g_go = 0;
void on_sigint(int) { g_stop = 1; }
void on_sigusr1(int) { g_go = 1; }

int replay(const char* path, bool verbose) {
  Stats st;
  digi::DigiEngine eng;
  const long n = digi::for_each_record(
      path, [&](std::uint64_t, const std::uint8_t* buf, std::uint32_t len) {
        ++st.records;
        if (len != digi::kWireSize || buf[0] != digi::kReportId) {
          ++st.other_id;  // not a 0x1A container: counted, not decoded
          return;
        }
        ++st.containers;
        digi::Fail why = digi::Fail::None;
        const auto f = digi::parse_container(buf, len, &why);
        if (!f) {
          ++st.parse_fail;  // a bad container rejects the whole frame
          return;
        }
        if (!f->matrix) {
          ++st.scene;  // scene frames carry no cell grid
          if (verbose) std::printf("scene ts16=%5u\n", f->ts16);
          return;
        }
        ++st.matrix;
        auto contacts = eng.process(f->cells);
        ++st.live_hist[contacts.size()];  // live contacts, before releases
        append_releases(eng, contacts, st.releases);
        if (verbose) {
          std::printf("matrix ts16=%5u tracks=%zu", f->ts16, contacts.size());
          for (const auto& c : contacts)
            std::printf(" [%u](%u,%u)%s%s", c.slot, c.x, c.y,
                        c.in_contact ? "" : "/rel",
                        c.confidence ? "" : "?");
          std::printf("\n");
        }
      });
  if (n < 0) {
    std::fprintf(stderr, "digid: cannot read %s\n", path);
    return 2;
  }
  std::printf(
      "digid replay : %s\nrecords=%ld containers(0x1A)=%ld matrix=%ld "
      "scene=%ld other-id=%ld parse_fail=%ld releases=%ld\n",
      path, st.records, st.containers, st.matrix, st.scene, st.other_id,
      st.parse_fail, st.releases);
  for (const auto& [live, frames] : st.live_hist)
    std::printf("  matrix frames with %u live track(s): %u\n", live, frames);
  // Pen-family pass over the same file, through the single-defined shape in
  // pen_replay.hpp.
  const pen::Stats ps = pen::replay_file(path);
  auto cons = [&](unsigned m) {
    auto it = ps.released.find(m);
    return it == ps.released.end() ? 0L : it->second;
  };
  std::printf(
      "pen replay    : frames=%ld walk-ok=%ld 0x2E-out=%ld walk-fail=%ld "
      "pen=%ld one=%ld fit=%ld cycle=%ld/%ld scene=%ld/%ld ticks=%ld "
      "rejected=%ld stops=%ld consumed={%ld,%ld,%ld,%ld,%ld,%ld} "
      "pos=%ld/%ld/%ld\n",
      ps.frames, ps.walk_ok, ps.out_grammar, ps.walk_fail, ps.pen_reports,
      ps.one_record, ps.mode_fit, ps.cycle_ok, ps.cycle_steps, ps.scene_pen,
      ps.scene_records, ps.ticks, ps.rejected, ps.held_incomplete, cons(6),
      cons(7), cons(8), cons(9), cons(10), cons(11), ps.pos.n, ps.pos.none,
      ps.pos.agree);
  return 0;
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
  // for the live touch front, "pci-<bdf>/digid-pen" for the live pen
  // front, empty for replay fronts).
  bool open_front(const char* phys = "", const FrontSpec& spec = kTouchSpec) {
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

int uhid_replay(const char* path, bool gate, double rate_hz) {
  // Rate cap on INPUT2 writes (--rate HZ, 0 = capture cadence): a replay is
  // a REAL event source for the desktop (libinput maps both fronts), and
  // full-cadence pen floods are hostile to tablet-protocol consumers. This
  // paces the
  // STREAM; it never changes what is decoded or sent.
  const long min_gap_us =
      rate_hz > 0.0 ? static_cast<long>(1e6 / rate_hz) : 0;
  auto throttle = [&](std::uint64_t& last) {
    if (!min_gap_us) return;
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    std::uint64_t now =
        std::uint64_t(ts.tv_sec) * 1000000u + ts.tv_nsec / 1000u;
    if (last != 0 && now > last && now - last < std::uint64_t(min_gap_us)) {
      const timespec s{0, long(min_gap_us - (now - last)) * 1000};
      nanosleep(&s, nullptr);
      now = last + std::uint64_t(min_gap_us);
    }
    last = now;
  };
  std::uint64_t last_pen_us = 0, last_mt_us = 0;
  UhidFront front, pfront;
  if (!front.open_front()) return 2;
  if (!pfront.open_front("", kPenSpec)) return 2;
  std::printf("digid: uhid fronts '%s' (0x045E:0x%04X) + '%s' "
              "(0x045E:0x%04X) created — verifying binding while feeding %s\n",
              digi::uhid::kName, digi::uhid::kProduct, digi::penhid::kName,
              digi::penhid::kProduct, path);
  // Wait up to 2 s for the kernel to bind and send START.
  for (int i = 0; i < 200 && !(front.started() && pfront.started()); ++i) {
    front.pump();
    pfront.pump();
    usleep(10000);
  }
  if (gate) {
    std::printf("digid: --gate — feed armed, send SIGUSR1 to start\n");
    while (!g_go && !g_stop) { front.pump(); pfront.pump(); usleep(10000); }
  }
  long releases = 0, sent = 0, pen_sent = 0;
  digi::DigiEngine eng;
  pen::Assembler pasm;
  pen::feed::PenFeed pfeed;
  std::uint64_t prev_mono = 0;
  bool have_prev = false;
  std::uint8_t payload[digi::uhid::kInputPayloadBytes];
  std::uint8_t ppayload[digi::penhid::kInputPayloadBytes];
  const long n = digi::for_each_record(
      path, [&](std::uint64_t mono, const std::uint8_t* buf, std::uint32_t len) {
        if (g_stop) return;
        // Replay at capture cadence, capping gaps at 100 ms, on every frame
        // of either family.
        if (have_prev && mono > prev_mono) {
          std::uint64_t us = mono - prev_mono;
          if (us > 100000) us = 100000;
          const timespec ts{0, static_cast<long>(us) * 1000};
          nanosleep(&ts, nullptr);
        }
        if (len != digi::kWireSize || buf[0] != digi::kReportId) {
          if (!is_pen_frame(buf[0])) return;
          prev_mono = mono;
          have_prev = true;
          const auto pf = pasm.decode(buf, len);
          if (pf.fail != pen::Fail::None) return;
          if (pfeed.update(pf)) {  // one report per full-profile frame
            digi::penhid::pack_report_0x01(ppayload, pfeed.report);
            throttle(last_pen_us);
            if (pfront.send_input(digi::penhid::kReportInput, ppayload,
                                  sizeof(ppayload)))
              ++pen_sent;
          }
          front.pump();
          pfront.pump();
          return;
        }
        prev_mono = mono;
        have_prev = true;
        digi::Fail why = digi::Fail::None;
        const auto f = digi::parse_container(buf, len, &why);
        if (!f || !f->matrix) return;  // scene frames produce no touch reports
        auto contacts = eng.process(f->cells);
        append_releases(eng, contacts, releases);
        digi::uhid::pack_report_0x0C(payload, sizeof(payload), f->ts16,
                                     contacts);
        throttle(last_mt_us);
        if (front.send_input(digi::uhid::kReportInput, payload,
                             sizeof(payload)))
          ++sent;
        front.pump();
        pfront.pump();
      });
  if (n < 0) {
    std::fprintf(stderr, "digid: cannot read %s\n", path);
    return 2;
  }
  std::printf("digid: feed done (mt-reports=%ld releases=%ld pen-reports=%ld)"
              ". Inspect the fronts (driver bind, /dev/input nodes); Ctrl-C "
              "to destroy.\n", sent, releases, pen_sent);
  while (!g_stop) { front.pump(); pfront.pump(); usleep(50000); }
  return 0;
}

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
          auto contacts = eng.process(f->cells);
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

// Per-frame CSV trace through the same decode the replay and the live path
// run (pen::replay_file with the sink).
// Measurement only — it ships no policy and never opens the device.
// Columns: us, rid, ts16(-1 none), have57, haveprof, half, pm(profile_mean),
// colsum, rowsum, cx/cy (the profile centroid, pen_position.hpp; -1 no full
// profile this frame), p2c/p2r (the controller's refined cell, 255 invalid),
// class5 (0x07 frame byte +5 raw; recorded, unused), ncon (released
// sub-frames), zc/zr (1 = that axis carries no signal, so its cx/cy is not a
// measurement; -1 = no full profile this frame), route + fcx/fcy (the
// SHIPPED feed's decision: which of the centroid / refined-cell / hold routes
// update() took, and the cell it chose BEFORE the map's clamp. Runs a real
// PenFeed alongside the decode:
// same header, same predicate, no copy).
int trace_pen(const char* path) {
  std::printf("us,rid,ts16,have57,haveprof,half,pm,colsum,rowsum,cx,cy,p2c,"
              "p2r,class5,ncon,zc,zer,route,fcx,fcy\n");
  long frames = 0;
  pen::feed::PenFeed feed;  // the shipped judge, driven frame-for-frame
  const std::function<void(std::uint64_t, const std::uint8_t*, std::uint32_t,
                           const pen::Frame&)> sink =
      [&](std::uint64_t us, const std::uint8_t* b, std::uint32_t,
          const pen::Frame& f) {
        ++frames;
        feed.update(f);  // judgement only: this tool never emits a report
        const pen::Profile& p = f.profile;
        const bool full = f.have_profile && p.full();
        const pen::PenPosition pos = full ? pen::locate_pen(p) : pen::PenPosition{};
        unsigned long cs = 0, rs = 0;
        for (const std::uint32_t v : p.col) cs += v;
        for (const std::uint32_t v : p.row) rs += v;
        std::printf(
            "%llu,0x%02X,%d,%d,%d,%d,%llu,%lu,%lu,%.4f,%.4f,%u,%u,%d,%zu,%d,"
            "%d,%c,%.4f,%.4f\n",
            (unsigned long long)us, b[0], f.have_pen_state ? int(f.pen_timestamp & 0xFFFF) : -1,
            int(f.have_pen_state), int(f.have_profile), int(p.half),
            (unsigned long long)pen::feed::profile_mean(p), cs, rs,
            full ? pos.col.cell : -1.0,
            full ? pos.row.cell : -1.0,
            p.refined_col, p.refined_row,
            b[0] == 0x07 ? int(b[5]) : -1, f.released.size(),
            full ? int(!pos.col.valid) : -1,
            full ? int(!pos.row.valid) : -1,
            feed.route, feed.cell_x, feed.cell_y);
      };
  const pen::Stats ps = pen::replay_file(path, &sink);
  if (ps.frames == 0) {
    std::fprintf(stderr, "digid: cannot read %s\n", path);
    return 2;
  }
  std::fprintf(stderr, "digid: trace-pen %s: %ld frames\n", path, frames);
  return 0;
}

int main(int argc, char** argv) {
  const char* replay_path = nullptr;
  const char* uhid_path = nullptr;
  const char* bdf = kBdfDefault;
  const char* record_path = nullptr;
  const char* trace_path = nullptr;
  bool verbose = false, gate = false, live_mode = false;
  double rate = 0.0;  // --rate HZ: uhid-replay INPUT2 cap (0 = capture cadence)
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--replay") && i + 1 < argc) replay_path = argv[++i];
    else if (!std::strcmp(argv[i], "--trace-pen") && i + 1 < argc)
      trace_path = argv[++i];
    else if (!std::strcmp(argv[i], "--uhid-replay") && i + 1 < argc)
      uhid_path = argv[++i];
    else if (!std::strcmp(argv[i], "--rate") && i + 1 < argc)
      rate = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--live")) live_mode = true;
    else if (!std::strcmp(argv[i], "--bdf") && i + 1 < argc) bdf = argv[++i];
    else if (!std::strcmp(argv[i], "--record") && i + 1 < argc) record_path = argv[++i];
    else if (!std::strcmp(argv[i], "--verbose") || !std::strcmp(argv[i], "-v"))
      verbose = true;
    else if (!std::strcmp(argv[i], "--gate")) gate = true;
    else {
      std::fprintf(stderr,
                   "usage: digid --replay FILE.binl [--verbose]\n"
                   "       digid --trace-pen FILE.binl\n"
                   "       digid --uhid-replay FILE.binl [--gate] [--rate HZ]\n"
                   "       digid --live [--bdf BDF] [--verbose] [--record FILE.binl]\n"
                   "  --live arms the digitizer port (SET_FEATURE [05 01]) and\n"
                   "   reverts it at teardown; --uhid-replay and --live carry\n"
                   "   both fronts ('Surface Digitizer Touch' + 'Surface Digitizer\n"
                   "   Pen'); --rate caps the replay INPUT2\n"
                   "   rate (a replay is a real desktop event source);\n"
                   "   the modes above run off captures\n"
                   "   only and never open the device.\n");
      return 2;
    }
  }
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  // SIGHUP too: the teardown revert is the only thing that releases the arm.
  // Closing the node does not: a digid killed by SIGHUP without a handler
  // left the port armed, with the firmware stream off and no client alive.
  std::signal(SIGHUP, on_sigint);
  std::signal(SIGUSR1, on_sigusr1);
  if (live_mode) return live(bdf, verbose, record_path);
  if (trace_path) return trace_pen(trace_path);
  if (uhid_path) return uhid_replay(uhid_path, gate, rate);
  if (replay_path) return replay(replay_path, verbose);
  std::fprintf(stderr, "digid: one of --replay/--uhid-replay/--live required\n");
  return 2;
}

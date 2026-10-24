// SPDX-License-Identifier: MIT
// heatd — touch processor for the Surface Laptop Studio 2 pad.
//
// Reads heat frames from the pad's hidraw node (or a capture), runs the
// contact engine, and presents the contacts as a PTP touchpad over uhid,
// using a collection of the device's own report descriptor.
#include <fcntl.h>
#include <linux/hidraw.h>
#include <linux/uhid.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "descwalk.hpp"
#include "devparams.hpp"
#include "engine.hpp"
#include "frame.hpp"
#include "heat_engine.hpp"
#include "ptp_collection.hpp"  // generated from the committed descriptor
#include "ptp_pack.hpp"

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

struct Options {
  const char* hidraw = nullptr;
  const char* replay = nullptr;
  const char* uhid_replay = nullptr;
  const char* script = nullptr;
  const char* device_params = nullptr;  // saved 0x06 response
  const char* engine = "heat";
  bool print_descriptor = false;
  bool trace = false;
  bool gate = false;
  // Microsoft vendor id with a product id distinct from the physical pad's
  // 0x0C46, so the front never shadows it.
  unsigned vendor = 0x045E;
  unsigned product = 0x4C46;
};

volatile sig_atomic_t g_stop = 0;
volatile sig_atomic_t g_go = 0;
void on_stop(int) { g_stop = 1; }
void on_go(int) { g_go = 1; }

// ---- front: report sizes derived from the descriptor fixture ----

struct Front {
  std::map<std::uint16_t, heat::ReportBits> bits;
  std::uint16_t input_0x81_bytes = 0;  // payload bytes, id byte excluded
  std::uint8_t contact_max = 0;        // TipSwitch count in report 0x81
};

Front front_from_fixture() {
  Front f;
  f.bits = heat::report_bits(ptp::collection, ptp::collection_size);
  auto it = f.bits.find(0x81);
  if (it != f.bits.end()) {
    f.input_0x81_bytes = static_cast<std::uint16_t>((it->second.input + 7) / 8);
    auto u = it->second.usages.find(heat::usage_key(0x0D, 0x42));
    if (u != it->second.usages.end())
      f.contact_max = static_cast<std::uint8_t>(u->second);
  }
  return f;
}

// GET_REPORT reply: zeros of the declared size, except 0x82 (contact max).
std::vector<std::uint8_t> feature_reply(std::uint16_t rid, const Front& f) {
  auto it = f.bits.find(rid);
  const std::uint32_t bytes = it == f.bits.end() ? 0 : (it->second.feature + 7) / 8;
  std::vector<std::uint8_t> v(bytes, 0);
  if (rid == 0x82 && !v.empty()) v[0] = f.contact_max;
  return v;
}

// ---- calibration and engine ----

heat::DeviceConfig config_from_file(const char* path) {
  std::ifstream in(path, std::ios::binary);
  std::vector<std::uint8_t> buf((std::istreambuf_iterator<char>(in)), {});
  if (in.bad()) {
    std::fprintf(stderr, "heatd: --device-params %s: read failed\n", path);
    std::exit(2);
  }
  heat::DeviceParams dp;
  std::string err;
  if (!heat::decode_device_params(buf.data(), buf.size(), &dp, &err)) {
    std::fprintf(stderr, "heatd: --device-params %s: %s (%zu B read)\n", path,
                 err.c_str(), buf.size());
    std::exit(2);
  }
  return dp.to_config();
}

void print_config(const heat::DeviceConfig& c, const char* source) {
  std::fprintf(stderr,
               "heatd: calibration: %dx%d him=%ux%u gate %u/%u/%u "
               "baseline=%u spawn<=%u (%s)\n",
               c.rows, c.cols, c.him_w, c.him_h, c.gate_lo, c.gate, c.gate_hi,
               c.baseline, c.spawn_gate, source);
}

// Calibration for modes without a device: --device-params or the defaults.
heat::DeviceConfig offline_config(const Options& opt) {
  if (opt.device_params) {
    auto c = config_from_file(opt.device_params);
    print_config(c, opt.device_params);
    return c;
  }
  heat::DeviceConfig c{};
  print_config(c, "built-in defaults");
  return c;
}

std::unique_ptr<heat::Engine> make_engine(const Options& opt,
                                          const heat::DeviceConfig& cfg) {
  const std::string_view e{opt.engine};
  if (e == "null") return std::make_unique<heat::NullEngine>();
  if (e == "heat") return std::make_unique<heat::HeatEngine>(cfg);
  std::fprintf(stderr, "heatd: unknown engine '%s'\n", opt.engine);
  std::exit(2);
}

// ---- per-frame pipeline shared by every mode ----

struct Counters {
  unsigned long frames = 0;
  unsigned long invalid = 0;
  unsigned long short_reads = 0;
  unsigned long lost = 0;  // frames implied missing by ts16 gaps
  unsigned long btn = 0;   // frames with the pad pressed
};

class Pipeline {
 public:
  explicit Pipeline(std::unique_ptr<heat::Engine> engine)
      : engine_(std::move(engine)),
        heat_(dynamic_cast<heat::HeatEngine*>(engine_.get())) {}

  // Parses one frame and runs the engine. Returns the frame's contacts with
  // the engine's release entries appended (up to kMaxContacts), or nothing
  // for an invalid frame.
  std::optional<std::vector<heat::Contact>> step(const std::uint8_t* buf,
                                                 std::size_t n) {
    heat::Fail why = heat::Fail::None;
    auto fr = heat::parse_frame(buf, n, &why);
    if (!fr) {
      ++count.invalid;
      last_fail = why;
      return std::nullopt;
    }
    if (have_ts_) count.lost += heat::ts_gap_frames(ts16, fr->ts16);
    ts16 = fr->ts16;
    have_ts_ = true;
    ++count.frames;
    auto contacts = engine_->process(*fr);
    if (engine_->button()) ++count.btn;
    if (heat_)
      for (auto& r : heat_->drain_releases())
        if (contacts.size() < heat::kMaxContacts) contacts.push_back(r);
    return contacts;
  }

  bool button() const { return engine_->button(); }
  const heat::HeatEngine* heat() const { return heat_; }

  Counters count;
  heat::Fail last_fail = heat::Fail::None;
  std::uint16_t ts16 = 0;  // of the last valid frame

 private:
  std::unique_ptr<heat::Engine> engine_;
  heat::HeatEngine* heat_;
  bool have_ts_ = false;
};

// ---- uhid front ----

class UhidOut {
 public:
  explicit UhidOut(const Front& f) : front_(f) {}
  ~UhidOut() { close(); }

  bool open(const Options& opt, bool nonblock, const char* phys = "") {
    fd_ = ::open("/dev/uhid", (nonblock ? O_NONBLOCK : 0) | O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
      std::fprintf(stderr, "heatd: open /dev/uhid: %s\n", std::strerror(errno));
      return false;
    }
    uhid_event ev{};
    ev.type = UHID_CREATE2;
    auto& c = ev.u.create2;
    std::snprintf(reinterpret_cast<char*>(c.name), sizeof(c.name), "%s",
                  "Surface Heat Processor");
    std::snprintf(reinterpret_cast<char*>(c.phys), sizeof(c.phys), "%s", phys);
    c.rd_size = static_cast<__u16>(ptp::collection_size);
    c.bus = 0x18;  // BUS_VIRTUAL
    c.vendor = opt.vendor;
    c.product = opt.product;
    c.version = 0x0001;
    c.country = 0;
    std::memcpy(c.rd_data, ptp::collection, ptp::collection_size);
    const std::size_t bytes =
        offsetof(uhid_event, u.create2.rd_data) + ptp::collection_size;
    if (::write(fd_, &ev, bytes) != static_cast<ssize_t>(bytes)) {
      std::fprintf(stderr, "heatd: UHID_CREATE2: %s\n", std::strerror(errno));
      return false;
    }
    created_ = true;
    return true;
  }

  void close() {
    if (fd_ < 0) return;
    if (created_) {
      uhid_event ev{};
      ev.type = UHID_DESTROY;
      if (::write(fd_, &ev, sizeof(ev)) < 0) {
        // closing the fd destroys the device as well
      }
    }
    ::close(fd_);
    fd_ = -1;
    created_ = false;
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

  bool send_0x81(std::uint16_t ts, const std::vector<heat::Contact>& contacts,
                 bool button) {
    std::vector<std::uint8_t> rep(front_.input_0x81_bytes, 0);
    heat::pack_report_0x81(rep.data(), rep.size(), ts, contacts, button);
    return send_input(0x81, rep.data(), rep.size());
  }

  // Handles one kernel event, if any. False on EOF or error.
  bool pump_one() {
    uhid_event ev;
    const ssize_t r = ::read(fd_, &ev, sizeof(ev));
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
    if (r != sizeof(ev)) return false;
    switch (ev.type) {
      case UHID_START:
        started_ = true;
        break;
      case UHID_STOP:
        started_ = false;
        break;
      case UHID_GET_REPORT: {
        const auto v = feature_reply(ev.u.get_report.rnum, front_);
        uhid_event rep{};
        rep.type = UHID_GET_REPORT_REPLY;
        rep.u.get_report_reply.id = ev.u.get_report.id;
        rep.u.get_report_reply.err = 0;
        rep.u.get_report_reply.size = static_cast<__u16>(v.size());
        if (!v.empty()) std::memcpy(rep.u.get_report_reply.data, v.data(), v.size());
        const std::size_t bytes = offsetof(uhid_event, u.get_report_reply.data) + v.size();
        if (::write(fd_, &rep, bytes) != static_cast<ssize_t>(bytes)) return false;
        break;
      }
      case UHID_SET_REPORT: {
        uhid_event rep{};  // the kernel waits for a reply
        rep.type = UHID_SET_REPORT_REPLY;
        rep.u.set_report_reply.id = ev.u.set_report.id;
        rep.u.set_report_reply.err = 0;
        if (::write(fd_, &rep, sizeof(rep)) < 0) return false;
        break;
      }
      default:
        break;
    }
    return true;
  }

  // Keeps the front serviced for d.
  void pump_for(Clock::duration d) {
    const auto until = Clock::now() + d;
    while (!g_stop && Clock::now() < until) {
      pump_one();
      std::this_thread::sleep_for(5ms);
    }
  }

  // Keeps the front serviced until SIGUSR1 or a stop signal.
  void wait_for_go(const char* what) {
    std::signal(SIGUSR1, on_go);
    std::fprintf(stderr, "heatd: armed; send SIGUSR1 (pid %d) to start %s\n",
                 static_cast<int>(getpid()), what);
    while (!g_stop && !g_go) {
      pump_one();
      std::this_thread::sleep_for(5ms);
    }
  }

  bool started() const { return started_; }
  int fd() const { return fd_; }

 private:
  const Front& front_;
  int fd_ = -1;
  bool created_ = false;
  bool started_ = false;
};

// Sends the frame's report when the front is up. A frame with no contacts is
// sent once, right after the last one that had some: it ends the kernel's
// pointer emulation, which release entries alone do not.
bool emit(UhidOut& uhid, std::uint16_t ts, const std::vector<heat::Contact>& contacts,
          bool button, bool& prev_nonempty) {
  bool sent = false;
  if (uhid.started() && (!contacts.empty() || prev_nonempty))
    sent = uhid.send_0x81(ts, contacts, button);
  prev_nonempty = !contacts.empty();
  return sent;
}

// ---- --replay: offline decode ----

void print_trace(const Pipeline& p, const std::vector<heat::Contact>& contacts) {
  std::printf("f%-6lu ts=%-6u", p.count.frames, p.ts16);
  const heat::HeatEngine* he = p.heat();
  if (he) std::printf(" k=%zu b=%zu", std::size_t(he->region_count()), he->last_blobs().size());
  for (const auto& ct : contacts)
    std::printf("  (%u,%u)#%u%s", ct.x, ct.y, ct.slot, ct.in_contact ? "" : "r");
  if (he)
    for (const auto& bl : he->last_blobs())
      std::printf("  [%u,%u,m%u,n%u]", unsigned(bl.cx + 0.5), unsigned(bl.cy + 0.5),
                  bl.min_cell, bl.cells_on);
  std::printf("\n");
}

int run_replay(const Options& opt, const Front& f) {
  std::ifstream in(opt.replay, std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "heatd: cannot open %s\n", opt.replay);
    return 2;
  }
  Pipeline p(make_engine(opt, offline_config(opt)));
  std::vector<std::uint8_t> buf(heat::kFrameSize);
  unsigned long contact_frames = 0;
  while (in) {
    in.read(reinterpret_cast<char*>(buf.data()), heat::kFrameSize);
    const auto got = static_cast<std::size_t>(in.gcount());
    if (got == 0) break;
    if (got < heat::kFrameSize) {
      ++p.count.short_reads;
      continue;
    }
    const auto contacts = p.step(buf.data(), got);
    if (!contacts) {
      std::fprintf(stderr, "frame %lu: invalid (reason %d)\n",
                   p.count.frames + p.count.invalid, static_cast<int>(p.last_fail));
      continue;
    }
    if (opt.trace) print_trace(p, *contacts);
    if (!contacts->empty()) ++contact_frames;
  }
  const Counters& c = p.count;
  std::printf(
      "replay %s: frames=%lu invalid=%lu short=%lu lost=%lu contact_frames=%lu (0x81 payload %u B,"
      " contact_max=%u) pressed=%lu\n",
      opt.replay, c.frames, c.invalid, c.short_reads, c.lost, contact_frames,
      f.input_0x81_bytes, f.contact_max, c.btn);
  return c.frames > 0 ? 0 : 1;
}

// ---- --uhid-replay: the front fed from a capture at 8 ms cadence ----

int run_uhid_replay(const Options& opt, const Front& f) {
  std::ifstream in(opt.uhid_replay, std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "heatd: cannot open %s\n", opt.uhid_replay);
    return 2;
  }
  UhidOut uhid(f);
  if (!uhid.open(opt, /*nonblock=*/true)) return 3;
  std::fprintf(stderr, "heatd: uhid front created; feed %s then inspect, "
                       "Ctrl-C to destroy\n", opt.uhid_replay);
  if (opt.gate) uhid.wait_for_go("feed");

  Pipeline p(make_engine(opt, offline_config(opt)));
  std::uint8_t buf[heat::kFrameSize];
  bool prev_nonempty = false, eof = false;
  auto next = Clock::now(), last_beat = Clock::now();
  while (!g_stop) {
    uhid.pump_one();
    if (Clock::now() - last_beat > 2s) {
      std::fprintf(stderr, "alive frames=%lu invalid=%lu\n", p.count.frames,
                   p.count.invalid);
      last_beat = Clock::now();
    }
    if (eof) {
      std::this_thread::sleep_for(50ms);
      continue;
    }
    in.read(reinterpret_cast<char*>(buf), heat::kFrameSize);
    if (static_cast<std::size_t>(in.gcount()) != heat::kFrameSize) {
      eof = true;  // keep serving the front until Ctrl-C
      continue;
    }
    const auto contacts = p.step(buf, sizeof(buf));
    if (!contacts) continue;
    emit(uhid, p.ts16, *contacts, p.button(), prev_nonempty);
    next = std::max(next, Clock::now()) + 8ms;
    std::this_thread::sleep_until(next);
  }
  std::fprintf(stderr, "heatd: uhid-replay exit frames=%lu invalid=%lu lost=%lu\n",
               p.count.frames, p.count.invalid, p.count.lost);
  return 0;
}

// ---- --script: the front fed from a scripted contact scenario ----
//
// One directive per line ('#' starts a comment):
//   c [id,x,y ...]   send one 0x81 with these contacts (none = 0-count)
//   r <n>            resend the previous contacts n more times
//   g <ms>           send nothing for ms
// Each frame sent is echoed to stdout as `s<idx> ts=<ts> id,x,y ...`.
// Always gated on SIGUSR1; the front stays up until Ctrl-C.
int run_script(const Options& opt, const Front& f) {
  std::ifstream in(opt.script);
  if (!in) {
    std::fprintf(stderr, "heatd: cannot open script %s\n", opt.script);
    return 2;
  }
  UhidOut uhid(f);
  if (!uhid.open(opt, /*nonblock=*/true)) return 3;
  std::fprintf(stderr, "heatd: uhid script front created from %s; Ctrl-C to destroy\n",
               opt.script);
  uhid.wait_for_go("scenario");

  std::uint16_t ts = 1;
  unsigned long sent = 0;
  std::vector<heat::Contact> cur;
  auto send = [&] {
    uhid.send_0x81(ts, cur, /*button=*/false);
    std::printf("s%lu ts=%u", ++sent, ts);
    for (const auto& ct : cur) std::printf(" %u,%u,%u", ct.slot, ct.x, ct.y);
    std::printf("\n");
    std::fflush(stdout);
    ++ts;
    uhid.pump_for(8ms);
  };
  auto fail = [](long ln, const char* what, const std::string& tok) {
    std::fprintf(stderr, "heatd: script:%ld: %s%s\n", ln, what, tok.c_str());
    return 4;
  };

  std::string line;
  long ln = 0;
  while (!g_stop && std::getline(in, line)) {
    ++ln;
    if (const auto h = line.find('#'); h != std::string::npos) line.erase(h);
    std::istringstream ls(line);
    std::string dir;
    if (!(ls >> dir)) continue;
    if (dir == "c") {
      cur.clear();
      for (std::string tok; ls >> tok;) {
        unsigned id = 0, x = 0, y = 0;
        if (std::sscanf(tok.c_str(), "%u,%u,%u", &id, &x, &y) != 3)
          return fail(ln, "bad contact '", tok + "'");
        cur.push_back(heat::Contact{
            .x = static_cast<std::uint16_t>(x), .y = static_cast<std::uint16_t>(y),
            .slot = static_cast<std::uint8_t>(id), .confidence = true,
            .in_contact = true});
      }
      send();
    } else if (dir == "r") {
      long n = 0;
      if (!(ls >> n) || n < 0) return fail(ln, "bad repeat", "");
      while (n-- > 0 && !g_stop) send();
    } else if (dir == "g") {
      long ms = 0;
      if (!(ls >> ms) || ms < 0) return fail(ln, "bad gap", "");
      uhid.pump_for(std::chrono::milliseconds(ms));
    } else {
      return fail(ln, "unknown directive '", dir + "'");
    }
  }
  std::fprintf(stderr, "heatd: uhid script done frames=%lu; serving until Ctrl-C\n", sent);
  while (!g_stop) {
    uhid.pump_one();
    std::this_thread::sleep_for(50ms);
  }
  return 0;
}

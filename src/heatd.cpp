// SPDX-License-Identifier: MIT
// heatd: reads the touchpad's heat frames and presents the contacts as a
// standard touchpad.
#include <fcntl.h>
#include <getopt.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
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
#include <string>
#include <string_view>
#include <vector>

#include "descwalk.hpp"
#include "devparams.hpp"
#include "engine.hpp"
#include "frame.hpp"
#include "heat_engine.hpp"
#include "hidraw_node.hpp"
#include "ptp_collection.hpp"
#include "ptp_pack.hpp"
#include "stream_guard.hpp"
#include "uhid_device.hpp"

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

struct Options {
  const char* hidraw = nullptr;
  const char* device_params = nullptr;
  const char* engine = "heat";
  bool print_descriptor = false;
  unsigned vendor = 0x045E;
  unsigned product = 0x4C46;
};

volatile sig_atomic_t g_stop = 0;
void on_stop(int) { g_stop = 1; }

struct Front {
  std::map<std::uint16_t, heat::ReportBits> bits;
  std::uint16_t input_0x81_bytes = 0;
  std::uint8_t contact_max = 0;
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

// Zeros of the declared size, except the contact count in 0x82.
std::vector<std::uint8_t> feature_reply(std::uint16_t rid, const Front& f) {
  auto it = f.bits.find(rid);
  const std::uint32_t bytes = it == f.bits.end() ? 0 : (it->second.feature + 7) / 8;
  std::vector<std::uint8_t> v(bytes, 0);
  if (rid == 0x82 && !v.empty()) v[0] = f.contact_max;
  return v;
}

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

std::unique_ptr<heat::Engine> make_engine(const Options& opt,
                                          const heat::DeviceConfig& cfg) {
  const std::string_view e{opt.engine};
  if (e == "null") return std::make_unique<heat::NullEngine>();
  if (e == "heat") return std::make_unique<heat::HeatEngine>(cfg);
  std::fprintf(stderr, "heatd: unknown engine '%s'\n", opt.engine);
  std::exit(2);
}

struct Counters {
  unsigned long frames = 0;
  unsigned long invalid = 0;
  unsigned long short_reads = 0;
  unsigned long lost = 0;  // frames implied missing by ts16 gaps
  unsigned long btn = 0;
};

class Pipeline {
 public:
  explicit Pipeline(std::unique_ptr<heat::Engine> engine)
      : engine_(std::move(engine)),
        heat_(dynamic_cast<heat::HeatEngine*>(engine_.get())) {}

  // The frame's contacts followed by its lifts; nothing for a bad frame.
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

  Counters count;
  heat::Fail last_fail = heat::Fail::None;
  std::uint16_t ts16 = 0;

 private:
  std::unique_ptr<heat::Engine> engine_;
  heat::HeatEngine* heat_;
  bool have_ts_ = false;
};

uhid::DeviceSpec front_spec(const Options& opt, const Front& f) {
  uhid::DeviceSpec spec;
  spec.name = "Surface Heat Processor";
  spec.vendor = std::uint16_t(opt.vendor);
  spec.product = std::uint16_t(opt.product);
  spec.descriptor = ptp::collection;
  spec.descriptor_size = ptp::collection_size;
  spec.feature_reply = [&f](std::uint8_t rid) { return feature_reply(rid, f); };
  return spec;
}

bool send_0x81(uhid::Device& front, const Front& f, std::uint16_t ts,
               const std::vector<heat::Contact>& contacts, bool button) {
  std::vector<std::uint8_t> rep(f.input_0x81_bytes, 0);
  heat::pack_report_0x81(rep.data(), rep.size(), ts, contacts, button);
  return front.send_input(0x81, rep.data(), rep.size());
}

// One empty report follows the last contact: lift reports alone don't end the
// kernel's pointer emulation.
bool emit(uhid::Device& front, const Front& f, std::uint16_t ts,
          const std::vector<heat::Contact>& contacts, bool button, bool& prev_nonempty) {
  bool sent = false;
  if (front.started() && (!contacts.empty() || prev_nonempty))
    sent = send_0x81(front, f, ts, contacts, button);
  prev_nonempty = !contacts.empty();
  return sent;
}

// Runs before the stream is armed, so no input reports get in the way.
bool read_device_params(int hdev, heat::DeviceConfig* cfg) {
  std::uint8_t buf[120] = {0x06};
  if (::ioctl(hdev, HIDIOCGFEATURE(sizeof(buf)), buf) < 0) {
    std::fprintf(stderr, "heatd: GET_FEATURE 0x06: %s — keeping built-in\n",
                 std::strerror(errno));
    return false;
  }
  heat::DeviceParams dp;
  std::string err;
  if (!heat::decode_device_params(buf, sizeof buf, &dp, &err)) {
    std::fprintf(stderr, "heatd: GET_FEATURE 0x06 decoded invalid: %s — "
                         "keeping built-in\n", err.c_str());
    return false;
  }
  *cfg = dp.to_config();
  return true;
}

// Switches the pad from boot-mouse reports to heat frames.
bool enable_heat_stream(int hdev) {
  std::uint8_t buf[2] = {0x05, 0x01};
  if (::ioctl(hdev, HIDIOCSFEATURE(sizeof(buf)), buf) < 0) {
    std::fprintf(stderr,
                 "heatd: SET_FEATURE 0x05=1: %s — stream may stay in "
                 "boot-mouse mode\n",
                 std::strerror(errno));
    return false;
  }
  return true;
}

std::string phys_for(const char* hidraw) {
  const std::string function = hidraw::pci_function(hidraw);
  return function.empty() ? std::string() : "pci-" + function + "/heatd";
}

int run_live(const Options& opt, const Front& f) {
  const int hdev = ::open(opt.hidraw, O_RDWR | O_CLOEXEC);
  if (hdev < 0) {
    std::fprintf(stderr, "heatd: open %s: %s\n", opt.hidraw, std::strerror(errno));
    return 2;
  }
  heat::DeviceConfig cfg{};
  const char* cfg_src = "built-in defaults";
  if (opt.device_params) {
    cfg = config_from_file(opt.device_params);
    cfg_src = opt.device_params;
  } else if (read_device_params(hdev, &cfg)) {
    cfg_src = "live GET 0x06";
  }
  print_config(cfg, cfg_src);
  enable_heat_stream(hdev);
  // A reset pad sends boot-mouse reports, all shorter than a heat frame.
  stream::Guard guard("heatd", [hdev] { return enable_heat_stream(hdev); });

  uhid::Device uhid;
  if (!uhid.open(front_spec(opt, f), phys_for(opt.hidraw))) {
    ::close(hdev);
    return 3;
  }

  Pipeline p(make_engine(opt, cfg));
  std::uint8_t buf[heat::kFrameSize];
  unsigned long contacts_total = 0, sent = 0;
  bool prev_nonempty = false;
  auto print_counters = [&] {
    const Counters& c = p.count;
    std::fprintf(stderr,
                 "heatd: frames=%lu invalid=%lu short=%lu lost=%lu "
                 "contacts=%lu sent=%lu ",
                 c.frames, c.invalid, c.short_reads, c.lost, contacts_total, sent);
  };
  auto next_beat = Clock::now();
  while (!g_stop) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(hdev, &rfds);
    FD_SET(uhid.fd(), &rfds);
    timeval tv{0, 200 * 1000};
    const int r = ::select(std::max(hdev, uhid.fd()) + 1, &rfds, nullptr, nullptr, &tv);
    if (r < 0) {
      if (errno == EINTR) continue;
      break;
    }
    guard.on_tick();
    if (Clock::now() >= next_beat) {
      print_counters();
      std::fprintf(stderr, "started=%d pressed=%lu\n", int(uhid.started()), p.count.btn);
      next_beat += 2s;
    }
    if (FD_ISSET(uhid.fd(), &rfds) && !uhid.pump()) {
      std::fprintf(stderr,
                   "heatd: uhid front read hit EOF/error (%s) - the kernel "
                   "removed or broke our device; exiting, counters below\n",
                   std::strerror(errno));
      break;
    }
    if (!FD_ISSET(hdev, &rfds)) continue;
    const ssize_t got = ::read(hdev, buf, sizeof(buf));
    if (got == 0) continue;
    if (got < 0) {
      if (errno == EINTR) continue;
      std::fprintf(stderr, "heatd: hidraw read: %s\n", std::strerror(errno));
      break;
    }
    if (static_cast<std::size_t>(got) < heat::kFrameSize) {
      ++p.count.short_reads;
      guard.on_default_report(got);
      continue;
    }
    const auto contacts = p.step(buf, static_cast<std::size_t>(got));
    if (!contacts) continue;
    contacts_total += contacts->size();
    if (emit(uhid, f, p.ts16, *contacts, p.button(), prev_nonempty)) ++sent;
  }
  uhid.close();
  ::close(hdev);
  print_counters();
  std::fprintf(stderr, "last_fail=%d pressed=%lu\n", int(p.last_fail), p.count.btn);
  return 0;
}

void usage(const char* argv0) {
  std::printf(
      "usage: %s --hidraw /dev/hidrawN | --print-descriptor  [options]\n"
      "  --hidraw NODE        read the pad, serve the uhid front\n"
      "  --engine heat|null   contact engine (default heat; null emits nothing)\n"
      "  --device-params FILE saved 0x06 response to calibrate from (read from\n"
      "                       the device itself when absent)\n"
      "  --vendor N, --product N  uhid front identity\n"
      "  --print-descriptor   dump the uhid front descriptor, exit\n",
      argv0);
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  enum : int { kHidraw = 256, kEngine, kDeviceParams, kVendor, kProduct, kPrintDescriptor };
  static const option kOptions[] = {
      {"hidraw", required_argument, nullptr, kHidraw},
      {"engine", required_argument, nullptr, kEngine},
      {"device-params", required_argument, nullptr, kDeviceParams},
      {"vendor", required_argument, nullptr, kVendor},
      {"product", required_argument, nullptr, kProduct},
      {"print-descriptor", no_argument, nullptr, kPrintDescriptor},
      {"help", no_argument, nullptr, 'h'},
      {},
  };
  auto number = [] { return static_cast<unsigned>(std::strtoul(optarg, nullptr, 0)); };
  for (int c; (c = getopt_long(argc, argv, "h", kOptions, nullptr)) != -1;) {
    switch (c) {
      case kHidraw: opt.hidraw = optarg; break;
      case kEngine: opt.engine = optarg; break;
      case kDeviceParams: opt.device_params = optarg; break;
      case kVendor: opt.vendor = number(); break;
      case kProduct: opt.product = number(); break;
      case kPrintDescriptor: opt.print_descriptor = true; break;
      case 'h': usage(argv[0]); return 0;
      default: usage(argv[0]); return 2;
    }
  }
  if (optind < argc) {
    std::fprintf(stderr, "heatd: unexpected argument %s\n", argv[optind]);
    usage(argv[0]);
    return 2;
  }
  if (opt.print_descriptor) {
    std::fwrite(ptp::collection, 1, ptp::collection_size, stdout);
    return 0;
  }
  const Front f = front_from_fixture();
  std::signal(SIGINT, on_stop);
  std::signal(SIGTERM, on_stop);
  if (opt.hidraw) return run_live(opt, f);
  std::fprintf(stderr, "heatd: nothing to do; try --help\n");
  usage(argv[0]);
  return 2;
}

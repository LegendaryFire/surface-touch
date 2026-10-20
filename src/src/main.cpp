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

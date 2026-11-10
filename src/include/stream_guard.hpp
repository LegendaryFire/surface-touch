// SPDX-License-Identifier: MIT
// Re-arms a device's stream after the device resets. A reset device comes
// back in its default mode with the hidraw node still open, so the reset has
// to be noticed: after a suspend, or from a report only the default mode
// sends.
#pragma once

#include <sys/types.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <utility>

namespace stream {

// Time the system has spent suspended: CLOCK_BOOTTIME advances across
// suspend, CLOCK_MONOTONIC does not.
inline std::int64_t suspended_ns() {
  timespec b{}, m{};
  clock_gettime(CLOCK_BOOTTIME, &b);
  clock_gettime(CLOCK_MONOTONIC, &m);
  return (std::int64_t(b.tv_sec) - m.tv_sec) * 1000000000 + (b.tv_nsec - m.tv_nsec);
}

class Guard {
 public:
  // `arm` returns whether the device accepted the stream mode.
  Guard(const char* name, std::function<bool()> arm)
      : name_(name), arm_(std::move(arm)) {}

  void on_tick() {
    const std::int64_t s = suspended_ns();
    if (s - slept_ < 1000000000) return;
    std::fprintf(stderr, "%s: resume after %.1f s suspended; re-arming stream\n",
                 name_, double(s - slept_) / 1e9);
    slept_ = s;
    arm();
  }

  void on_default_report(ssize_t len) {
    if (blocked_ || Clock::now() - last_arm_ < std::chrono::seconds(1)) return;
    std::fprintf(stderr, "%s: %zd-byte report outside the stream (device reset); "
                         "re-arming stream\n", name_, len);
    arm();
  }

 private:
  using Clock = std::chrono::steady_clock;

  void arm() {
    last_arm_ = Clock::now();
    blocked_ = !arm_();
    if (blocked_)
      std::fprintf(stderr, "%s: re-arm failed; restart %s to recover\n", name_, name_);
  }

  const char* name_;
  std::function<bool()> arm_;
  std::int64_t slept_ = suspended_ns();
  Clock::time_point last_arm_ = Clock::now();
  bool blocked_ = false;
};

}  // namespace stream

// SPDX-License-Identifier: MIT
// A virtual HID device over /dev/uhid.
#pragma once

#include <fcntl.h>
#include <linux/uhid.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace uhid {

struct DeviceSpec {
  std::string name;
  std::uint16_t vendor = 0, product = 0;
  const std::uint8_t* descriptor = nullptr;
  std::size_t descriptor_size = 0;
  // GET_REPORT answer for a feature report id (empty when it has none).
  std::function<std::vector<std::uint8_t>(std::uint8_t)> feature_reply;
};

class Device {
 public:
  Device() = default;
  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;
  ~Device() { close(); }

  // Creates the device; `phys` becomes its phys string. Errors go to stderr.
  bool open(const DeviceSpec& spec, const std::string& phys) {
    spec_ = spec;
    fd_ = ::open("/dev/uhid", O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) return fail("open /dev/uhid");
    uhid_event ev{};
    ev.type = UHID_CREATE2;
    auto& c = ev.u.create2;
    std::snprintf(reinterpret_cast<char*>(c.name), sizeof c.name, "%s", spec_.name.c_str());
    std::snprintf(reinterpret_cast<char*>(c.phys), sizeof c.phys, "%s", phys.c_str());
    c.rd_size = static_cast<__u16>(spec_.descriptor_size);
    c.bus = 0x18;  // BUS_VIRTUAL
    c.vendor = spec_.vendor;
    c.product = spec_.product;
    c.version = 0x0001;
    std::memcpy(c.rd_data, spec_.descriptor, spec_.descriptor_size);
    if (!write_event(ev, offsetof(uhid_event, u.create2.rd_data) + spec_.descriptor_size))
      return fail("UHID_CREATE2");
    created_ = true;
    return true;
  }

  // Destroys the device. Closing the fd would destroy it as well.
  void close() {
    if (fd_ < 0) return;
    if (created_) {
      uhid_event ev{};
      ev.type = UHID_DESTROY;
      write_event(ev, sizeof ev);
    }
    ::close(fd_);
    fd_ = -1;
    created_ = started_ = false;
  }

  bool send_input(std::uint8_t report_id, const std::uint8_t* payload, std::size_t n) {
    uhid_event ev{};
    ev.type = UHID_INPUT2;
    ev.u.input2.size = static_cast<__u16>(n + 1);
    ev.u.input2.data[0] = report_id;
    std::memcpy(ev.u.input2.data + 1, payload, n);
    return write_event(ev, offsetof(uhid_event, u.input2.data) + n + 1);
  }

  // Handles every pending kernel event. False once the device is gone.
  bool pump() {
    for (;;) {
      uhid_event ev;
      const ssize_t r = ::read(fd_, &ev, sizeof ev);
      if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return true;
      if (r != static_cast<ssize_t>(sizeof ev)) return false;
      if (!handle(ev)) return false;
    }
  }

  bool started() const { return started_; }  // a driver has bound the device
  int fd() const { return fd_; }

 private:
  bool handle(const uhid_event& ev) {
    switch (ev.type) {
      case UHID_START: started_ = true; return true;
      case UHID_STOP: started_ = false; return true;
      case UHID_GET_REPORT: {
        const std::vector<std::uint8_t> v =
            spec_.feature_reply ? spec_.feature_reply(ev.u.get_report.rnum)
                                : std::vector<std::uint8_t>{};
        uhid_event rep{};
        rep.type = UHID_GET_REPORT_REPLY;
        rep.u.get_report_reply.id = ev.u.get_report.id;
        rep.u.get_report_reply.size = static_cast<__u16>(v.size());
        if (!v.empty()) std::memcpy(rep.u.get_report_reply.data, v.data(), v.size());
        return write_event(rep, offsetof(uhid_event, u.get_report_reply.data) + v.size());
      }
      case UHID_SET_REPORT: {  // the kernel waits for a reply
        uhid_event rep{};
        rep.type = UHID_SET_REPORT_REPLY;
        rep.u.set_report_reply.id = ev.u.set_report.id;
        return write_event(rep, sizeof rep);
      }
      default: return true;
    }
  }

  bool write_event(const uhid_event& ev, std::size_t bytes) {
    return ::write(fd_, &ev, bytes) == static_cast<ssize_t>(bytes);
  }

  bool fail(const char* what) {
    std::fprintf(stderr, "%s: %s: %s\n", program_invocation_short_name, what,
                 std::strerror(errno));
    return false;
  }

  DeviceSpec spec_;
  int fd_ = -1;
  bool created_ = false;
  bool started_ = false;
};

}  // namespace uhid

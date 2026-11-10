// SPDX-License-Identifier: MIT
// A hidraw node's HID identity and the PCI function it belongs to.
#pragma once

#include <linux/hidraw.h>
#include <sys/ioctl.h>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <regex>
#include <string>

namespace hidraw {

struct Identity {
  std::uint16_t vendor = 0, product = 0;
};

inline std::optional<Identity> identity(int fd) {
  hidraw_devinfo info{};
  if (::ioctl(fd, HIDIOCGRAWINFO, &info) < 0) return std::nullopt;
  return Identity{std::uint16_t(info.vendor), std::uint16_t(info.product)};
}

// "0000:00:10.6", or empty if the node is not on PCI.
inline std::string pci_function(const std::string& node) {
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dev = fs::canonical(node, ec);
  if (ec) return {};
  const fs::path sys =
      fs::canonical(fs::path("/sys/class/hidraw") / dev.filename() / "device", ec);
  if (ec) return {};
  static const std::regex kPciAddress("[0-9a-f]{4}:[0-9a-f]{2}:[0-9a-f]{2}\\.[0-7]");
  std::string function;
  for (const fs::path& part : sys)
    if (std::regex_match(part.string(), kPciAddress)) function = part.string();
  return function;
}

}  // namespace hidraw

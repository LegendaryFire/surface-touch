// SPDX-License-Identifier: MIT
// Facts about a hidraw node: its HID identity (from the node itself) and
// the PCI function it sits on (from sysfs). The node may be given through a
// udev symlink such as /dev/surface-thc-pad.
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

// HIDIOCGRAWINFO on an open node.
inline std::optional<Identity> identity(int fd) {
  hidraw_devinfo info{};
  if (::ioctl(fd, HIDIOCGRAWINFO, &info) < 0) return std::nullopt;
  return Identity{std::uint16_t(info.vendor), std::uint16_t(info.product)};
}

// The PCI function the node's device hangs off ("0000:00:10.6"): the
// innermost PCI address in its sysfs path. Empty if there is none.
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

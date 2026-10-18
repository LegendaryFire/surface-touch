# surface-touch

Touch input for the **Microsoft Surface Laptop Studio 2** on Linux.

The Laptop Studio 2 hangs its touch hardware off two Intel THC SPI
functions (`8086:51D0`, `8086:51D1`). The kernel's `intel_quickspi`
driver does not bind those IDs today, so on a stock kernel the touch
hardware is inert. This package will carry:

* `module/` — the Intel THC kernel driver as a DKMS package with the
  Raptor Lake-L THC SPI IDs enabled,
* the userspace processors that turn the pad and digitiser streams into
  standard HID input devices,
* udev/systemd glue and packaging.

Status: work in progress; the kernel side lands first.

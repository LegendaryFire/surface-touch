# Changelog

## Unreleased

### Added
- Palm rejection on the touchpad and the touchscreen. A touched region whose
  area per contact core reaches 700 mm2 is a palm; fingers held together stay
  fingers because each brings its own core. Palms are reported with
  Confidence = 0, which the kernel passes on as a palm tool, so libinput
  ignores them instead of seeing a lift or a tap. A contact stays a palm
  until it lifts.
- Touchscreen: while the pen is in range, and for 0.5 s after, every touch is
  a palm (the writing hand resting on the screen).

### Fixed
- The touchpad was often dead after waking from suspend: the driver resets
  the controller on resume, which drops it out of the heat stream, and
  `heatd` kept reading the same node without selecting the stream again.
  `heatd` now re-selects the heat stream once per detected suspend, and on a
  report that is not a heat frame (a device-initiated reset).

### Removed
- The developer modes of `heatd` (`--replay`, `--uhid-replay`, `--script`,
  `--trace`, `--gate`) and `digid` (`--replay`, `--uhid-replay`,
  `--trace-pen`, `--gate`, `--rate`). The services never used them.
  `digid --live --record FILE` still saves a session.

### Changed
- Documentation: resume resets the touch controllers but keeps the HID
  devices and their hidraw nodes; the earlier text said the nodes are
  re-created under new numbers on every resume.

## 1.0.0 — 2026-11-07

First production release.

### Added
- `surface-thc-hid` DKMS package: Intel THC driver (`intel-thc`,
  `intel-quickspi`, `intel-quicki2c`) with Raptor Lake-L THC SPI support —
  binds `8086:51D0` (digitiser) and `8086:51D1` (touchpad) at boot. See
  NOTICE for the change set.
- `heatd` — touchpad heat-container processor exposing a PTP touchpad over
  uhid (hid-multitouch/libinput standard). Reads the device's own
  calibration (feature 0x06) at open; selects the heat stream per open
  (feature 0x05).
- `digid` — digitiser processor: arms the port per open and reverts on
  teardown, exposes `Surface Digitizer Touch` (MT) and `Surface Digitizer Pen`.
- udev rules matching the HID identity (never the hidraw number), stable
  `/dev/surface-thc-pad` and `/dev/surface-thc-digitizer` symlinks, and
  device-triggered `surface-heatd` / `surface-digid` services that re-resolve
  the node and re-arm the port whenever they restart.
- modprobe configuration keeping the third-party `ithc` driver off the THC
  functions.
- Arch `PKGBUILD` (+ install hooks) and a distribution-neutral
  `make install` / `make uninstall`.

### Known limitations
- Very light pen strokes can miss: libinput tips pressure pens at ≥5 % of
  range while this device reports decoded pressure honestly from 0.
- Pen position arrives at the wire's 66.6 Hz maximum; drawn curves can look
  slightly faceted.
- Pen barrel/eraser buttons and tilt are not reported — the wire carries no
  data for them.
- Wake-on-touch and runtime suspend of the ports are not implemented.
- ADL-S/ADL-M PCI IDs are carried from the validated tree but untested on
  hardware.

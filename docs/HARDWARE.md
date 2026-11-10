# Hardware

## Supported machine

The **Microsoft Surface Laptop Studio 2** (Intel Raptor Lake-L, validated on
BIOS 23.101.143). Everything here was built and accepted on that machine;
sibling Alder Lake platforms with the same THC SPI IDs may work (the module
carries ADL-S/ADL-M IDs from the validated tree — untested).

## The touch hardware

| PCI function | ID | Holds | HID identity | Processor |
|---|---|---|---|---|
| `00:10.6` | `8086:51D0` | touchscreen + pen | `045e:0c5d` ("quickspi-hid 045E:0C5D") | `digid` |
| `00:10.7` | `8086:51D1` | touchpad | `045e:0c46` ("quickspi-hid 045E:0C46") | `heatd` |

Nothing in this package hardcodes those addresses: udev finds the transports
by the HID identity above and names their nodes; the processors read the PCI
function from sysfs only to label their virtual devices, never the other way
round, so the stack follows the hardware wherever the BIOS puts it.

Both ports speak HIDSPI over the Intel Touch Host Controller. Once the pad
is switched into its streaming mode (feature `0x05` = 1, claimed **per
open**) it emits only vendor heat containers (~1000-byte frames, ~125 Hz);
the digitiser at ~4350-byte containers + pen-family frames (~100 Hz).
Nothing in the kernel decodes those containers — that is what `heatd` and
`digid` are.

## The devices you will see

Input devices the desktop uses (created over uhid by the processors):

| Name | Type | Provided by |
|---|---|---|
| `Surface Heat Processor` | PTP touchpad | heatd |
| `Surface Digitizer Touch` | direct touch MT screen | digid |
| `Surface Digitizer Pen` | pen | digid |

Inert nodes that stay visible (harmless — they emit nothing while the
processors run): the pad's kernel node `quickspi-hid 045E:0C46 Touchpad`
and the digitiser's firmware `Touchscreen`/`Stylus` nodes. The digitiser
firmware nodes go silent while the port is armed by design: the armed
stream and the kernel single-pointer path are mutually exclusive modes of
the same port.

The stack's own uhid devices carry distinct IDs (`045e:4c46`, `045e:4d54`,
`045e:5045`) so they never collide with the physical `0c46`/`0c5d` or with
each other in udev rules or desktop device lists.

## Sleep and wake

This platform has **s2idle only**. Across suspend the firmware powers the
touch controllers down — ACPI `_PS3` asserts the panel reset line unless
wake is armed — so on resume the driver **resets** them (the
`reset_on_resume` path). The HID devices and their `/dev/hidrawN` nodes
persist across the reset; the controllers come back in their default report
mode.

Consequences built into this package:

* `heatd` and `digid` re-select their stream after a resume, and
  whenever a report arrives that only the default mode sends (a
  device-initiated reset), on the node they already have open. For the
  digitizer those are its native single-contact Touch Screen report (`0x40`,
  6 bytes) and its Pen report (`0x01`).
* **Never** open a hard-coded `/dev/hidrawN`: the numbers are assigned at
  enumeration and differ between boots. Use `/dev/surface-thc-pad` /
  `/dev/surface-thc-digitizer`.
* Each service is bound to its node's device unit: it stops when the node
  disappears and udev starts it again when the node returns
  (`Restart=always` covers a processor that exits on its own). Arming is
  part of every open, so a restarted processor leaves the same correct state
  a first-start one does.
* Nothing else on the system should pin these node numbers either (custom
  `udev` rules, X/Wayland quirks configs, permissions rules keyed on the
  kernel name). Match on `HID_ID`/`MODALIAS`/the symlink instead.

## Kernels and signing

* Validated with Arch's `linux-surface` 7.2.x and built against 7.2.15 and
  7.3 headers. Any 7.x kernel with matching headers should build.
* DKMS modules ship unsigned. With Secure Boot enabled, load fails with
  `Required key not available` — sign via `dkms` MOK support (`SIGN_MOK=yes`
  in `dkms.conf`, then `mokutil --import`) or run with Secure Boot off.
* After a kernel upgrade the DKMS hook rebuilds on install of the new
  kernel's headers; if the hook did not fire: `sudo dkms autoinstall`.

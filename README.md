# surface-touch

Touch input for the **Microsoft Surface Laptop Studio 2** on Linux: touchpad,
touchscreen and pen, end to end.

The Laptop Studio 2 hangs its touch hardware off two Intel Touch Host
Controller (THC) SPI functions (`8086:51D0`, `8086:51D1`). The kernel's
`intel_quickspi` driver does not bind those IDs today, so on a stock kernel
the touch hardware is inert — and the device streams data the stock stack
cannot decode even once bound. This package is the complete fix:

* **`module/`** — the Intel THC kernel driver as a DKMS package, with Raptor
  Lake-L THC SPI support folded in: it binds both PCI functions at boot.
* **`heatd`** — the touchpad processor. The pad streams vendor "heat"
  containers; `heatd` computes contacts from them and exposes a standard
  precision touchpad (PTP) over `uhid`, which `hid-multitouch` and
  libinput drive natively: two-finger scroll, pinch, tap, click-to-drag —
  everything the desktop expects.
* **`digid`** — the digitizer processor. It arms the touchscreen port,
  decodes the same container family for touch and pen, and exposes a proper
  direct-touch MT device and a pen device, again over `uhid`.
* **udev rules and systemd services** that follow the hardware through
  suspend/resume automatically (see [Sleep and wake](#sleep-and-wake)).

See [NOTICE](NOTICE) for the provenance of every component.

## What works

| Input | State |
|---|---|
| Touchpad: pointer, multi-finger gestures, scroll, pinch, tap, click | works, daily-driver quality |
| Touchscreen: multi-touch, pinch-zoom, five-finger gestures | works |
| Pen: pointer tracking, hover, tip contact | works — smooth and aligned |
| Pen: graded pressure | works |
| Pen: barrel/eraser buttons, tilt | not reported — the wire carries no data for them |
| Wake on touch, auto-suspend of the ports | not supported (future work) |

## Requirements

* Surface Laptop Studio 2 (the only machine this is validated on; the module
  also carries other Alder Lake THC SPI IDs — untested, see [NOTICE](NOTICE)).
* Linux 7.2 or 7.3 with matching **kernel headers**; `dkms`, `gcc`, `make`,
  `python3`, and **OpenCV ≥ 5** (Arch: `pacman -S dkms opencv`).
* The `linux-surface` kernel is recommended on a Surface (it adds the
  keyboard/touch-button/aggregator drivers), but surface-touch itself is not
  surface-kernel specific and has no dependency on it.
* **Secure Boot**: DKMS modules here are unsigned. Boot with Secure Boot off,
  or sign the modules with your own MOK (`dkms` supports `SIGN_MOK`) before
  enabling it.

**Conflicts** — remove or mask these before installing:

* `iptsd` — the old third-party daemon. Its rules and units will race ours.
* `ithc` — claimed by this package's modprobe configuration
  (`/etc/modprobe.d/surface-touch.conf`) so it cannot steal the ports.

## Install

### Any distribution (manual)

```bash
make                                   # builds heatd + digid
sudo make install                      # defaults to PREFIX=/usr/local
# packaging installs (files under /usr, udev/modprobe in /usr/lib):
# sudo make install PREFIX=/usr UDEVDIR=/usr/lib/udev/rules.d MODPROBEDIR=/usr/lib/modprobe.d
```

### Arch Linux (native package)

```bash
make dist                              # → dist/surface-touch-1.0.0.tar.gz
mkdir -p ../surface-touch-pkg
cp PKGBUILD surface-touch.install dist/surface-touch-1.0.0.tar.gz ../surface-touch-pkg/
cd ../surface-touch-pkg && makepkg -si
```

The PKGBUILD builds the daemons and installs the DKMS module source; the
module itself is compiled on the target machine by DKMS at package-install
time, so it follows kernel upgrades.

`make install` installs the binaries, the DKMS module (and builds it for the
running kernel), the udev rules, the systemd units and the modprobe
configuration, then reloads udev/systemd. On a kernel update, DKMS rebuilds
the module automatically via its install hook; if your DKMS lacks hooks, run
`sudo dkms autoinstall` after upgrading the kernel.

### Verify

```bash
lspci -nnk -s 00:10.                       # both functions: driver in use: intel_quickspi
systemctl status surface-heatd.service surface-digid.service   # active (running)
ls -l /dev/surface-thc-pad /dev/surface-thc-digitizer       # stable symlinks exist
```

Then touch. The touchpad node the desktop uses is named
`Surface Heat Processor`, the touchscreen `Surface Digitizer Touch` and the pen
`Surface Digitizer Pen` (`libinput list-devices | grep -A2 Surface`).

## How it works

```
  pad (heat stream) ──► /dev/hidrawN (intel_quickspi) ──► heatd ──► uhid "Surface Heat Processor" ──► hid-multitouch ──► libinput
  digitizer        ──► /dev/hidrawM (intel_quickspi) ──► digid ─► uhid "Surface Digitizer Touch" + "Surface Digitizer Pen"
```

The daemons never talk the vendor protocol on the desktop's behalf: they
translate the vendor stream into standards-compliant HID devices, and the
ordinary kernel/desktop stack (hid-multitouch, libinput, your compositor)
does the rest. The stream mode each port is put into is claimed **per
device open** and released on teardown, so nothing about the hardware stays
changed when the services stop.

## Sleep and wake

On this platform the firmware powers the touch controllers down across
suspend, so on resume the driver **resets them**. The HID devices and their
`/dev/hidrawN` nodes stay, but a reset controller comes back in its default
report mode:

* `heatd` notices the resume (or the first report outside the heat stream)
  and selects the heat stream again on the same open node;
* the digitizer keeps its stream mode across the reset, so `digid`
  carries on without a re-arm;
* if a node does go away, the processor exits and its service restarts it:
  the services resolve the node by the device's own HID identity
  (`045e:0c46`, `045e:0c5d`) and arm the port on the fresh open. The udev
  rules give the nodes stable symlinks, `/dev/surface-thc-pad` and
  `/dev/surface-thc-digitizer`, so nothing depends on a hidraw number, which
  is assigned at enumeration and can differ between boots.

A finger on the glass a second after opening the lid is the normal
experience. If it is ever not, `systemctl restart surface-heatd` is the whole
fix, and `docs/TROUBLESHOOTING.md` has the rest.

## Uninstall

```bash
sudo make uninstall     # or: pacman -Rns surface-touch, then reboot
```

The ports return to driver-unclaimed after a reboot with nothing else
installed; nothing else on the machine is touched.

## Known limitations

* **Very light pen strokes can miss** — libinput tips a pressure-capable
  pen at ≥5 % of the pressure range while this device reports the decoded
  value honestly from 0, so the faintest contacts can register as hover
  with a tip flag but no click. Decoding pressure from a floor is the
  honest trade until the threshold behaviour is solved upstream.
* **Drawn pen curves look slightly faceted** — the controller delivers pen
  positions at 66.6 Hz and that is the wire's maximum; nothing port-side
  can raise it.
* Pen barrel/eraser buttons and tilt are not reported: the wire carries no
  data for them. They are omitted honestly
  rather than invented.
* The digitizer's firmware-provided `Touchscreen`/`Stylus` input nodes go
  silent while `digid` is armed — that is the mode contract, not a
  fault: the armed stream *is* the truth, and the uhid devices replace them.
* A kernel input device named `quickspi-hid 045E:0C46 Touchpad` (and the
  digitizer firmware nodes) stays visible while the stack runs; it is inert
  and emits nothing.

## Reporting issues

Open an issue with the output of `lspci -nnk -s 00:10.`,
`journalctl -u surface-heatd -u surface-digid -b`, `uname -a` and what you
expected vs saw.

## License

* `module/` (kernel): GPL-2.0-only — see [NOTICE](NOTICE) for provenance and
  `LICENSES/GPL-2.0-only.txt`.
* `src/` daemons, scripts, rules, units: MIT — `LICENSES/MIT.txt`.
* `CHANGELOG.md` records releases; `docs/` covers hardware, troubleshooting
  and development.

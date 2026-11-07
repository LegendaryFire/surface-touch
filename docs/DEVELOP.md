# Development

## Layout

```
module/        DKMS package (dkms.conf + drivers/hid/intel-thc-hid sources).
               Built at install time against the target kernel headers;
               out-of-tree build also works directly:
                 make -C module KDIR=/lib/modules/$(uname -r)/build
src/           heatd (pad) + digid (digitiser). `make` in src/
               builds both.
scripts/       resolver + service entry points (path tokens @BINDIR@/
               @LIBEXECDIR@ are substituted by the top-level install).
udev/          90-surface-touch.rules — identity matching, symlinks, triggers.
systemd/       the two services (device-triggered, restart-churn-tolerant).
modprobe.d/    ithc exclusion.
Makefile       build / install / uninstall / dist (tarball).
PKGBUILD       Arch packaging; surface-touch.install carries the DKMS hooks.
```

## Design decisions worth knowing before you change things

* **Match identity, never node number.** hidraw numbers are assigned at
  enumeration and differ between boots. Rules, scripts and units key on
  `045e:0c46` / `045e:0c5d`. Any new rule you write must too.
* **Resume re-creates the hidraw nodes.** The firmware drops the touch
  controllers across suspend and the driver's `reset_on_resume` path resets
  them; the nodes come back under new numbers, so anything holding a node
  must re-resolve it after a resume.
* **Arming is per open.** The heat-stream mode (pad, feature `0x05`) and the
  digitiser stream mode (feature `0x05` `[05 01]`) are claimed on every
  device open and never inherited across opens; `digid` reverts with
  `[05 00]` at teardown (SIGINT/SIGTERM/SIGHUP all route there — a digid
  killed without reverting leaves the port armed until the controller
  resets). Keep any new daemon in that
  discipline: claim on open, revert on exit.
* **The services are udev-triggered and never installed.** No `[Install]`
  section: `SYSTEMD_WANTS` from `90-surface-touch.rules` starts them when the
  device appears, and `Restart=always` with `StartLimitIntervalSec=0`
  restarts a processor whose node went away.
* **The uhid fronts carry distinct identities** (`045e:4c46` pad front,
  `045e:4d54` touch front, `045e:5045` pen front, distinct phys strings).
  They must never shadow a physical ID or each other in udev rules or
  desktop device lists.
* **Descriptors are fixtures, not hand-written.** The pad front's descriptor
  is a byte-exact collection of the device's own report descriptor
  (`src/fixtures/`, generated into `src/build/ptp_collection.hpp` by
  `src/tools/bin2hpp.py`). The digitiser touch front's descriptor is the
  standard MT-class collection carrying this device's geometry;
  the pen front is the device's own pen collection kept verbatim. Never
  hand-edit any of them.
* **The engine sources are pinned.** `heatd-sources.sha256` pins the heatd
  engine sources; `make identity` checks them on every `digid` link.

## Kernel module loop

```bash
make -C module KDIR=/lib/modules/$(uname -r)/build
sudo dkms remove -m surface-thc-hid -v 1.0.0 --all
sudo make install          # reinstalls + rebuilds via dkms
```

Keep `module/` byte-honest: it is the tree the hardware was validated with
(plus the documented upstream 7.3 forward-port).

## Releasing

1. Bump `VERSION`, `module/dkms.conf` `PACKAGE_VERSION`, `PKGBUILD`
   `pkgver`, and add the `CHANGELOG.md` entry.
2. `make clean && make` and a real
   session: pad gestures, two-finger pinch on the touchscreen, pen hover →
   tip → drag, then a suspend/resume cycle with the pointer working on the
   first touch after the lid opens.
3. `make dist`; sign the tarball; point the PKGBUILD's `sha256sums` at it.
   The released set is `surface-touch-<ver>.tar.gz` + `PKGBUILD` +
   `surface-touch.install` (the PKGBUILD consumes the tarball as its source);
   `makepkg -si` from that directory gives the installable Arch package.

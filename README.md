# surface-touch

Touchpad, touchscreen and pen support for the Surface Laptop Studio 2 on
Linux.

The touch hardware sits on two Intel THC SPI ports that the stock
`intel_quickspi` driver doesn't bind, and once bound it sends data nothing
in the kernel decodes. This repo has three parts:

- `module/`: the THC driver with support for this machine, built by DKMS.
- `heatd`: turns the touchpad's raw data into a standard touchpad.
- `digid`: does the same for the touchscreen and pen.

Gestures, scrolling and the rest come from the normal Linux stack:
hid-multitouch, libinput and your desktop.

## Status

Touchpad, multi-touch touchscreen, pen position and pressure all work, and
survive suspend. Not supported: pen buttons, eraser and tilt (the hardware
doesn't send them), and wake on touch.

Only tested on the Surface Laptop Studio 2.

## Requirements

- Linux 7.2 or 7.3 with matching kernel headers
- dkms, make, GCC 15 or newer, OpenCV 5 (`pacman -S dkms opencv` on Arch)
- Secure Boot off, or sign the module with your own key; it's unsigned

Remove `iptsd` if you have it. This package blocks `ithc` from the ports.

## Install

```
make
sudo make install
```

On Arch you can build a package instead:

```
make dist
cp PKGBUILD surface-touch.install dist/surface-touch-*.tar.gz /some/dir
cd /some/dir && makepkg -si
```

DKMS rebuilds the module on kernel updates. If it didn't, run
`sudo dkms autoinstall`.

To check it works:

```
lspci -nnk -s 00:10.        # both ports: driver in use: intel_quickspi
systemctl status surface-heatd surface-digid
```

The desktop sees three devices: `Surface Heat Processor` (touchpad),
`Surface Digitizer Touch` and `Surface Digitizer Pen`.

To remove: `sudo make uninstall` (or `pacman -R surface-touch`), then reboot.

## When something's wrong

- Touch stopped working: `systemctl restart surface-heatd surface-digid`.
- Ports not bound to `intel_quickspi`: the module didn't build for your
  kernel. Install the matching headers and run `sudo dkms autoinstall`.
  `Required key not available` in dmesg means Secure Boot.
- `ithc` bound instead: `/etc/modprobe.d/surface-touch.conf` is missing.
  Reinstall and reboot.
- Touchpad frozen with `-110` timeouts in dmesg: the controller is wedged.
  Reboot.

When reporting a problem, include `uname -a`, `lspci -nnk -s 00:10.` and
`journalctl -b -u surface-heatd -u surface-digid`.

## Known issues

- Very light pen strokes may not draw. libinput needs 5% pressure before
  the pen counts as touching.
- Pen curves can look slightly angular: the hardware only reports the pen
  at 66 Hz.
- The kernel's own touchpad, touchscreen and stylus devices stay listed but
  do nothing while the services run.

## Development

```
module/        the THC driver (DKMS)
src/           heatd and digid
tools/         heatview, a live view of the touchpad heat map
udev/          names the devices and starts the services
systemd/       the two services
modprobe.d/    keeps ithc off the ports
```

`make tools` builds `tools/heatview/surface-heatview`. It needs SDL3 and read
access to the pad (`sudo setfacl -m u:$USER:r /dev/surface-thc-pad`), and
shows the heat map with what heatd makes of it. `--record FILE` saves the
frames.

Commits that passed a real session on the machine are tagged
`hw-validated-<date>`. To see what hasn't been tested on hardware yet:

```
git diff $(git describe --tags --match 'hw-validated-*' --abbrev=0) -- src/
```

To release: bump `VERSION`, `PACKAGE_VERSION` in `module/dkms.conf` and
`pkgver` in the PKGBUILD. Test touchpad, touchscreen, pen and a
suspend/resume on the machine, tag the commit, then `make dist` and put the
tarball's checksum in the PKGBUILD.

## License

`module/` is GPL-2.0-only, everything else MIT. See `NOTICE` and
`LICENSES/`.

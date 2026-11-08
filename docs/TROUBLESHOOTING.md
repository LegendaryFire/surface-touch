# Troubleshooting

Work top to bottom; each step states what it proves.

## 1. Is the driver bound?

```bash
lspci -nnk -s 0000:00:10. | grep -A3 '51d[01]'
dkms status | grep surface-thc-hid
```

`Kernel driver in use: intel_quickspi` on **both** `51d0` and `51d1` is the
healthy answer.

* **`intel_quickspi` not in use, function unclaimed** — the DKMS module did
  not build/install for the running kernel. `dkms status` should show
  `surface-thc-hid/1.0.0, $(uname -r), installed`; if not, install the kernel
  headers package matching your kernel (`linux-headers`,
  `linux-surface-headers`), then `sudo dkms autoinstall` (or reinstall the
  package). A `Required key not available` in `dmesg` means Secure Boot and
  unsigned modules — see `docs/HARDWARE.md`.
* **`ithc` in use instead** — the modprobe configuration is missing or was
  overridden; `/etc/modprobe.d/surface-touch.conf` (or the package copy under
  `/usr/lib/modprobe.d/`) must be present, then reboot. An old `iptsd`
  install will also fight for the ports — remove it.

## 2. Are the processors running?

```bash
systemctl status surface-heatd.service surface-digid.service
journalctl -b -u surface-heatd -u surface-digid | tail -40
ls -l /dev/surface-thc-pad /dev/surface-thc-digitizer
```

* `active (running)` + a `serving /dev/hidrawN` line in the journal is the
  healthy answer. The node number is irrelevant — it is re-resolved every
  start, and can differ between boots.
* **restarting with "no … hidraw node (045e:0c46) within 15 s"** — the pad
  transport did not enumerate. If step 1 was fine, check `dmesg -T |
  grep -i quickspi` for probe errors: `Get ACPI resources failed` means the
  BIOS/ACPI did not mark the port HIDSPI (check for a firmware update; the
  Surface ships with it marked).
* **touchpad dead after a resume** — the journal should show `heatd:
  resume after … s suspended; re-arming stream`. If that line is missing or
  followed by `re-arm failed`, `systemctl restart surface-heatd`.
* **crash-loop after a resume** — give it one cycle: the services restart
  on a 2-second cadence while the node settles. Then
  `systemctl restart surface-heatd` manually and watch the journal.

## 3. Desktop sees nothing

```bash
libinput list-devices | grep -B1 -A3 Surface
```

The three `Surface …` devices must appear; only the touchpad one must move the
pointer. If `Surface Heat Processor` is there but inert, confirm heatd's
journal shows `S1 calibration:` and a rising `frames=` counter in the
2-second heartbeats while you drag.

## 4. Pad dies on boot with `hidraw read: Input/output error`

There is a known boot race: the pad's own descriptor can load
`hid-multitouch` ~0.4 s after the port is claimed, which re-binds the pad
and destroys the hidraw node `heatd` opened (dmesg shows `Wrong input
report length: 15` around it). The service exits and is restarted a moment
later — it recovers by design. Persistent failure at this exact point is a
bug: file a report with the journal.

## 5. Everything worked, then the pad froze and dmesg shows `-110`

`timeout for waiting TxDMA completion` / `Wait Get Report Response
timeout` marks a wedged controller (rare, after heavy bind/rebind churn;
cause unresolved upstream). Reboot. Rebinding the PCI
function can also clear it:

```bash
echo 0000:00:10.7 | sudo tee /sys/bus/pci/drivers/intel_quickspi/unbind
echo 0000:00:10.7 | sudo tee /sys/bus/pci/drivers/intel_quickspi/drivers_probe
```

**Never** bind a PCI function with `driver_override`/forced `drivers_probe`
for a device that is not in the module's ID table — `quickspi_probe()`
dereferences the matched ID's driver data; a forced bind is a kernel
crash, not a failed probe.

## 6. Pen behaviour questions

* **Very light strokes don't draw**: known limit — libinput tips a
  pressure-capable pen at ≥5 % of range, and this device reports decoded
  pressure honestly from 0, so the faintest contacts can stay hover. Press
  a touch firmer; a real fix is open upstream.
* **Drawn curves look faceted**: the wire delivers pen positions at 66.6 Hz
  and nothing port-side can raise that.
* **Barrel/eraser buttons and tilt do nothing**: the wire carries no data
  for them; they are deliberately not invented.

## 7. Building from source fails

* `missing dependency: OpenCV` — install OpenCV **5** (`opencv` on Arch);
  OpenCV 4 is not supported (cv::moments moved modules in 5).
* `modpost`/` pahole version differs` **warnings** during the module build
  are cosmetic when the build exits 0.
* Headers mismatch (`/lib/modules/$(uname -r)/build` missing) — install the
  headers package for the exact running kernel.

## 8. Getting out of a bad state

* `sudo systemctl stop surface-heatd surface-digid` — the ports return to
  their unarmed mode on teardown; the digitiser firmware nodes start
  emitting single-pointer events again.
* Uninstalling (README) leaves the machine as the plain kernel state after a
  reboot; nothing in the package persists firmware or hardware state across
  reboots.

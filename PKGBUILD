# SPDX-License-Identifier: MIT
# Maintainer: tristan <tristan.balon@outlook.com>
# Touch stack for the Microsoft Surface Laptop Studio 2: Intel THC QuickSPI
# kernel module with Raptor Lake-L support (DKMS) + heatd/digid input
# processors, systemd units and udev rules.
pkgname=surface-touch
pkgver=1.0.0
pkgrel=1
pkgdesc="Touchpad, touchscreen and pen support for the Surface Laptop Studio 2 (Intel THC QuickSPI + heatd/digid)"
arch=('x86_64')
url="https://github.com/LegendaryFire/surface-touch"
license=('GPL-2.0-only' 'MIT')
# renamed from sls2-touch; replacing it pulls in the old package's removal hooks
provides=('sls2-touch')
conflicts=('sls2-touch')
depends=('dkms' 'opencv' 'glibc' 'gcc-libs')
makedepends=('gcc' 'make' 'pkgconf' 'sed' 'grep' 'systemd')
optdepends=('linux-surface-headers: matching kernel headers when running the linux-surface kernel')
install=surface-touch.install
source=("$pkgname-$pkgver.tar.gz")
sha256sums=('SKIP')   # fill from `make dist` output when publishing

build() {
	make -C "$srcdir/$pkgname-$pkgver"
}

package() {
	make -C "$srcdir/$pkgname-$pkgver" install \
		DESTDIR="$pkgdir" \
		PREFIX=/usr \
		UDEVDIR=/usr/lib/udev/rules.d \
		MODPROBEDIR=/usr/lib/modprobe.d
	install -D -m644 "$srcdir/$pkgname-$pkgver/LICENSES/GPL-2.0-only.txt" "$pkgdir/usr/share/licenses/$pkgname/GPL-2.0-only.txt"
	install -D -m644 "$srcdir/$pkgname-$pkgver/LICENSES/MIT.txt" "$pkgdir/usr/share/licenses/$pkgname/MIT.txt"
}

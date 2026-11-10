# SPDX-License-Identifier: MIT
# surface-touch — touch stack for the Microsoft Surface Laptop Studio 2
#
#   make                 build heatd + digid
#   sudo make install    install, then build the kernel module with dkms and
#                        reload udev/systemd
#   sudo make uninstall  undo install
#   make dist            source tarball in dist/
#
# Packagers set DESTDIR (and usually PREFIX, UDEVDIR, MODPROBEDIR); with
# DESTDIR set only files are installed, and the package's install hooks do
# the dkms/udev/systemd part (see surface-touch.install).
#
# VERSION must match module/dkms.conf PACKAGE_VERSION and the PKGBUILD.

NAME    := surface-touch
VERSION := $(shell cat VERSION)
MODULE  := surface-thc-hid

DESTDIR     ?=
PREFIX      ?= /usr/local
BINDIR      ?= $(PREFIX)/bin
LIBDIR      ?= $(PREFIX)/lib
UNITDIR     ?= $(LIBDIR)/systemd/system
UDEVDIR     ?= /etc/udev/rules.d
MODPROBEDIR ?= /etc/modprobe.d
DOCDIR      ?= $(PREFIX)/share/doc/$(NAME)
MODSRCDIR   ?= /usr/src/$(MODULE)-$(VERSION)

UNITS := surface-heatd.service surface-digid.service

# Files a kernel build leaves in module/; never installed or shipped.
MODULE_BUILD_FILES := '*.o' '*.ko' '*.ko.zst' '*.mod' '*.mod.c' '.*.cmd' \
                      Module.symvers modules.order modules.builtin.modinfo

# Fills in the paths the systemd units refer to.
SUBST := sed -e 's|@BINDIR@|$(BINDIR)|g' -e 's|@DOCDIR@|$(DOCDIR)|g'

# ---- build -----------------------------------------------------------------

all:
	$(MAKE) -C src

clean:
	$(MAKE) -C src clean

# ---- install ---------------------------------------------------------------

install: install-files
uninstall: uninstall-files
ifeq ($(DESTDIR),)
install: register
uninstall: unregister
endif

install-files: all
	install -Dm0755 -t $(DESTDIR)$(BINDIR) src/heatd src/digid
	install -d $(DESTDIR)$(UNITDIR)
	$(foreach u,$(UNITS),$(SUBST) systemd/$(u) > $(DESTDIR)$(UNITDIR)/$(u);)
	install -Dm0644 -t $(DESTDIR)$(UDEVDIR) udev/90-surface-touch.rules
	install -Dm0644 -t $(DESTDIR)$(MODPROBEDIR) modprobe.d/surface-touch.conf
	install -Dm0644 -t $(DESTDIR)$(DOCDIR) README.md NOTICE CHANGELOG.md
	cp -a LICENSES docs $(DESTDIR)$(DOCDIR)/
	@# the module sources dkms builds from: a fresh copy, without build output
	rm -rf $(DESTDIR)$(MODSRCDIR)
	install -d $(dir $(DESTDIR)$(MODSRCDIR))
	cp -a module $(DESTDIR)$(MODSRCDIR)
	find $(DESTDIR)$(MODSRCDIR) \( $(foreach f,$(MODULE_BUILD_FILES),-name $(f) -o) -false \) -delete

uninstall-files:
	rm -f $(DESTDIR)$(BINDIR)/heatd $(DESTDIR)$(BINDIR)/digid
	rm -f $(addprefix $(DESTDIR)$(UNITDIR)/,$(UNITS))
	rm -f $(DESTDIR)$(UDEVDIR)/90-surface-touch.rules
	rm -f $(DESTDIR)$(MODPROBEDIR)/surface-touch.conf
	rm -rf $(DESTDIR)$(DOCDIR) $(DESTDIR)$(MODSRCDIR)
	rm -rf $(DESTDIR)$(LIBDIR)/$(NAME)    # helper scripts installed by 1.0.0,
	rm -rf $(DESTDIR)$(LIBDIR)/sls2-touch # and by 1.0.0 under the old name

# Live system only: build the module and make udev/systemd pick up the
# new rules and units for devices that are already present.
register: install-files
	@command -v dkms >/dev/null || { echo "dkms not found: module NOT installed (install dkms, then re-run)"; exit 1; }
	dkms remove -m $(MODULE) -v $(VERSION) --all 2>/dev/null || true
	dkms add -m $(MODULE) -v $(VERSION)
	dkms install -m $(MODULE) -v $(VERSION)
	systemctl daemon-reload
	udevadm control --reload-rules
	udevadm trigger --subsystem-match=hidraw --action=add || true
	@echo "installed.  check:  systemctl status $(UNITS)"

unregister: uninstall-files
	command -v dkms >/dev/null && dkms remove -m $(MODULE) -v $(VERSION) --all || true
	systemctl daemon-reload || true
	udevadm control --reload-rules || true
	@echo "uninstalled; a reboot returns the machine to the plain kernel state"

# ---- dist ------------------------------------------------------------------

dist: clean
	@mkdir -p dist
	tar --exclude=./dist --exclude=./.git --exclude=./.idea \
	    $(addprefix --exclude=,$(MODULE_BUILD_FILES)) \
	    --transform 's,^\.,$(NAME)-$(VERSION),' \
	    -czf dist/$(NAME)-$(VERSION).tar.gz .
	@echo "dist/$(NAME)-$(VERSION).tar.gz"

.PHONY: all clean install uninstall install-files uninstall-files register unregister dist

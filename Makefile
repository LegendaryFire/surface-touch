# surface-touch — touch stack for the Microsoft Surface Laptop Studio 2
#
# make                    build heatd + digid (the kernel module builds
#                         inside module/ via dkms at install time)
# sudo make install       build + install + dkms install + reload udev/systemd
# sudo make uninstall     reverse of install (dkms remove included)
# make dist               self-contained source tarball in dist/
#
# Packaging variables (defaults suit a manual install; the PKGBUILD and any
# distro packager override them):
#   PREFIX=/usr  UDEVDIR=/usr/lib/udev/rules.d  MODPROBEDIR=/usr/lib/modprobe.d
# Keep VERSION in step with module/dkms.conf PACKAGE_VERSION and PKGBUILD.

NAME     := surface-touch
VERSION  := $(shell cat VERSION)
MODULE   := surface-thc-hid

DESTDIR  ?=
PREFIX   ?= /usr/local
BINDIR   ?= $(PREFIX)/bin
LIBDIR   ?= $(PREFIX)/lib
UNITDIR  ?= $(LIBDIR)/systemd/system
UDEVDIR  ?= /etc/udev/rules.d
MODPROBEDIR ?= /etc/modprobe.d
DOCDIR   ?= $(PREFIX)/share/doc/$(NAME)
USRSRCDIR ?= /usr/src/$(MODULE)-$(VERSION)

# runtime paths written into the installed units and scripts
RUN_BINDIR     := $(BINDIR)
RUN_LIBEXECDIR := $(LIBDIR)/$(NAME)
RUN_DOCDIR     := $(DOCDIR)

SED = sed -e 's|@BINDIR@|$(RUN_BINDIR)|g' \
        -e 's|@LIBEXECDIR@|$(RUN_LIBEXECDIR)|g' \
        -e 's|@DOCDIR@|$(RUN_DOCDIR)|g'

.PHONY: all clean install uninstall dist

all:
	$(MAKE) -C src

clean:
	$(MAKE) -C src clean

install: all
	install -d $(DESTDIR)$(BINDIR)
	install -m 0755 src/heatd src/digid $(DESTDIR)$(BINDIR)/
	install -d $(DESTDIR)$(LIBDIR)/$(NAME)
	install -m 0755 scripts/surface-resolve-hidraw $(DESTDIR)$(LIBDIR)/$(NAME)/
	$(SED) scripts/surface-run-heatd   > $(DESTDIR)$(LIBDIR)/$(NAME)/surface-run-heatd
	$(SED) scripts/surface-run-digid > $(DESTDIR)$(LIBDIR)/$(NAME)/surface-run-digid
	chmod 0755 $(DESTDIR)$(LIBDIR)/$(NAME)/surface-run-heatd \
	           $(DESTDIR)$(LIBDIR)/$(NAME)/surface-run-digid
	install -d $(DESTDIR)$(UNITDIR)
	$(SED) systemd/surface-heatd.service   > $(DESTDIR)$(UNITDIR)/surface-heatd.service
	$(SED) systemd/surface-digid.service > $(DESTDIR)$(UNITDIR)/surface-digid.service
	install -D -m 0644 udev/90-surface-touch.rules $(DESTDIR)$(UDEVDIR)/90-surface-touch.rules
	install -D -m 0644 modprobe.d/surface-touch.conf $(DESTDIR)$(MODPROBEDIR)/surface-touch.conf
	install -d $(DESTDIR)$(DOCDIR)
	install -m 0644 README.md NOTICE CHANGELOG.md $(DESTDIR)$(DOCDIR)/
	cp -a LICENSES $(DESTDIR)$(DOCDIR)/
	cp -a docs $(DESTDIR)$(DOCDIR)/
	rm -rf $(DESTDIR)$(USRSRCDIR)
	install -d "$(dir $(DESTDIR)$(USRSRCDIR))"
	cp -a module $(DESTDIR)$(USRSRCDIR)
	find $(DESTDIR)$(USRSRCDIR) -name '.*.cmd' -delete; \
	find $(DESTDIR)$(USRSRCDIR) \( -name '*.o' -o -name '*.ko' -o -name '*.ko.zst' \
	  -o -name '*.mod' -o -name '*.mod.c' -o -name Module.symvers \
	  -o -name modules.order -o -name modules.builtin.modinfo \) -delete
ifeq ($(DESTDIR),)
	@# live system (not a package stage): register the module, then make
	@# the new rules/units/services take effect for the devices already here
	command -v dkms >/dev/null || { echo "dkms not found: module NOT installed (install dkms, then re-run)"; exit 1; }
	dkms remove -m $(MODULE) -v $(VERSION) --all 2>/dev/null || true
	dkms add    -m $(MODULE) -v $(VERSION)
	dkms install -m $(MODULE) -v $(VERSION)
	systemctl daemon-reload
	udevadm control --reload-rules
	udevadm trigger --subsystem-match=hid --action=add || true
	udevadm trigger --subsystem-match=hidraw || true
	@echo "installed.  check:  systemctl status surface-heatd.service surface-digid.service"
endif

uninstall:
	rm -f $(DESTDIR)$(BINDIR)/heatd $(DESTDIR)$(BINDIR)/digid
	rm -rf $(DESTDIR)$(LIBDIR)/$(NAME)
	rm -f $(DESTDIR)$(UNITDIR)/surface-heatd.service $(DESTDIR)$(UNITDIR)/surface-digid.service
	rm -f $(DESTDIR)$(UDEVDIR)/90-surface-touch.rules
	rm -f $(DESTDIR)$(MODPROBEDIR)/surface-touch.conf
	rm -rf $(DESTDIR)$(DOCDIR)
	rm -rf $(DESTDIR)$(USRSRCDIR)
ifeq ($(DESTDIR),)
	command -v dkms >/dev/null && dkms remove -m $(MODULE) -v $(VERSION) --all || true
	systemctl daemon-reload || true
	udevadm control --reload-rules || true
	@echo "uninstalled; a reboot returns the machine to the plain kernel state"
endif

dist: clean
	@mkdir -p dist
	tar --exclude='./dist' --exclude='./.git' --exclude='./src/build' \
	    --exclude='./src/heatd' --exclude='./src/digid' \
	    --transform 's,^\.,$(NAME)-$(VERSION),' \
	    -czf dist/$(NAME)-$(VERSION).tar.gz .
	@echo "dist/$(NAME)-$(VERSION).tar.gz"

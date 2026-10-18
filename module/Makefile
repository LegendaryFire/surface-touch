# Out-of-tree build of drivers/hid/intel-thc-hid.
#
# The kernel Makefile in that directory keys off CONFIG_INTEL_* which are not
# set for an out-of-tree build, so pass them on the command line.

KDIR ?= /lib/modules/$(shell uname -r)/build
MDIR := $(CURDIR)/drivers/hid/intel-thc-hid

KCONF := CONFIG_INTEL_THC_HID=m CONFIG_INTEL_QUICKSPI=m CONFIG_INTEL_QUICKI2C=m

all modules:
	$(MAKE) -C $(KDIR) M=$(MDIR) $(KCONF) modules

clean:
	$(MAKE) -C $(KDIR) M=$(MDIR) clean

.PHONY: all modules clean

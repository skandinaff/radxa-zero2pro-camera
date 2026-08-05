#
# SPDX-License-Identifier: GPL-2.0
#
# imx415.c is vendored verbatim from mainline kernel tag v6.3
# (drivers/media/i2c/imx415.c) -- see PORT_NOTES.md in the parent build/
# directory for what, if anything, needed changing to build it against
# our 6.1.68 target (v6.3 is only 2 minor versions ahead).
#

PWD := $(CURDIR)
KDIR ?= /usr/src/linux-headers-$(shell uname -r)

obj-m += imx415.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# This comes from the zio Makefile

LINUX ?= /lib/modules/$(shell uname -r)/build

mcuio-y := core.o bus.o
regmap-mcuio-y := regmap-mcuio-remote.o

obj-m = mcuio.o mcuio-hc-drv.o regmap-mcuio.o mcuio-hc-dev.o


GIT_VERSION = $(shell cd $(src); git describe --dirty --long --tags)

# WARNING: the line below doesn't work in-kernel if you compile with O=
EXTRA_CFLAGS += -I$(obj)/include/ -DGIT_VERSION=\"$(GIT_VERSION)\"

all: modules

modules:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd)

modules_install:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) $@


coccicheck:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) coccicheck


.PHONY: tools

# this make clean is ugly, I'm aware...
clean:
	rm -rf `find . -name \*.o -o -name \*.ko -o -name \*~ `
	rm -rf `find . -name Module.\* -o -name \*.mod.c`
	rm -rf .tmp_versions modules.order


# SPDX-License-Identifier: GPL-2.0

VERSION := bes2600_0.3.10_2022.1125

obj-m := bes2600.o

bes2600-objs := ap.o \
		bes2600_cfgvendor.o \
		bes2600_factory.o \
		bes2600_log.o \
		bes2600_sdio.o \
		bes_chardev.o \
		bes_fw.o \
		bes_fw_common.o \
		bh.o \
		debug.o \
		epta_coex.o \
		epta_request.o \
		fwio.o \
		hwio.o \
		main.o \
		pm.o \
		queue.o \
		scan.o \
		sta.o \
		tx_loop.o \
		txrx.o \
		txrx_opt.o \
		wsm.o

ccflags-y += -DBES2600_DRV_VERSION=\"$(VERSION)\"

.PHONY: modules clean

modules:
	@$(MAKE) -C $(KERNEL) M=$(CURDIR) modules

clean:
	@$(MAKE) -C $(KERNEL) M=$(CURDIR) clean

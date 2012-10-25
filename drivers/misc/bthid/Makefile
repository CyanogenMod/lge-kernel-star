#  Copyright (C) 2009-2011 Broadcom Corporation
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License, version 2, as published by
#  the Free Software Foundation (the "GPL").
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  A copy of the GPL is available at http://www.broadcom.com/licenses/GPLv2.php,
#  or by writing to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
#  Boston, MA  02111-1307, USA.
#
#
# Makefile for Broadcom bthid Driver
#

# Set KDIR to location of your kernel source
KDIR := `pwd`/../../../../kernel

# Prebuilt toolchain directory
PREBUILT := `pwd`/../../../../prebuilt

# Set compiler toolchain
CCOMPILER := $(PREBUILT)/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-

obj-m := bthid.o

scsdio-y := bthid.o

.PHONY: all clean

all:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules ARCH=arm CROSS_COMPILE=$(CCOMPILER)

clean:
	@rm -rf *~ .depend *.o .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers

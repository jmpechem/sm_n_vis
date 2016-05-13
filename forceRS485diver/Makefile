#makefile for XR17v35x PCIe UARTs for Linux 2.6.32 and newer
KERNEL_SRC = /lib/modules/`uname -r`/build

all: build

obj-m += xr17v35x.o

xrpci-objs :=	xr17v35x.o

EXTRA_CFLAGS += -DDEBUG=1

build:
	$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) modules

install:
	cp xr17v35x.ko /lib/modules/$(shell uname -r)/kernel/drivers/char
clean:
	rm -f *~ *.o
	rm -f *~ *.ko

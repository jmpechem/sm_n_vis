# makefile for XR17v35x PCIe UARTs for Linux 2.6.32 and newer
# modified by Suhan Park (psh117@gmail.com)
# -- Bug fixed: driver load on boot time
# -- You must uncheck 8250 pci driver before kernel build (Linux Kernel 3.4 and newer)

KERNEL_SRC = /lib/modules/`uname -r`/build

all: build

obj-m += xr17v35x.o

xrpci-objs :=	xr17v35x.o

EXTRA_CFLAGS += -DDEBUG=1

build:
	$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) modules

install:
	install -p -m 644 xr17v35x.ko /lib/modules/$(shell uname -r)/kernel/drivers/char
	echo "xr17v35x" >> /etc/modules
clean:
	rm -f *~ *.o
	rm -f *~ *.ko

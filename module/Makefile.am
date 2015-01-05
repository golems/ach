# See Also: Kbuild.in

all: ach.ko

module_src = ach_linux.c ach_linux.h ach_impl_linux.h

ach.ko: $(module_src)
	make -C $(KDIR) M=$(PWD) modules

# # linux kbuild can't uninstall, so we install via automake
#
# install-data-hook:
# 	make -C $(KDIR) M=$(PWD) modules_install

linuxmodulesdir=/lib/modules/$(KERNELRELEASE)/extra
linuxmodules_DATA=ach.ko
install-data-hook:
	depmod

clean-local:
	make -C $(KDIR) M=$(PWD) clean
	rm -f $(module_src)

## Copy files to support out-of-tree builds
ach_linux.c: $(top_srcdir)/src/module/ach_linux.c
	cp $(top_srcdir)/src/module/ach_linux.c .
ach_linux.h: $(top_srcdir)/src/module/ach_linux.h
	cp $(top_srcdir)/src/module/ach_linux.h .
ach_impl_linux.h: $(top_srcdir)/src/module/ach_impl_linux.h
	cp $(top_srcdir)/src/module/ach_impl_linux.h .

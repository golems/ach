# See Also: Kbuild.in

all: ach.ko

module_src = ach_klinux.c ach_klinux.h ach_impl_klinux.h ach_impl_common.h

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
ach_klinux.c: $(top_srcdir)/src/module/ach_klinux.c
	cp $(top_srcdir)/src/module/ach_klinux.c .
ach_klinux.h: $(top_srcdir)/src/module/ach_klinux.h
	cp $(top_srcdir)/src/module/ach_klinux.h .
ach_impl_klinux.h: $(top_srcdir)/src/module/ach_impl_klinux.h
	cp $(top_srcdir)/src/module/ach_impl_klinux.h .
ach_impl_common.h: $(top_srcdir)/include/ach_impl_common.h
	cp $(top_srcdir)/include/ach_impl_common.h .

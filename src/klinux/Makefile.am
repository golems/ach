# See Also: Kbuild.in

EXTRA_DIST = ach_klinux.c README.module dkms/Kbuild

if BUILD_KBUILD

ach.ko: ach_klinux.c $(module_headers)
	if test "x$(builddir)" != "x$(srcdir)"; then cp -v $(srcdir)/ach_klinux.c $(builddir); fi
	make -C $(KDIR) M=$(PWD) V=0 modules

hdir = $(top_srcdir)/include
module_headers = $(hdir)/ach_klinux.h $(hdir)/ach/generic.h $(hdir)/ach/klinux_generic.h $(hdir)/ach/private_klinux.h $(hdir)/ach/private_generic.h $(hdir)/ach/impl_generic.h

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
endif

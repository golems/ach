
if BUILD_DKMS

install-data-local:
	dkms add -m ach-klinux -v @PACKAGE_VERSION@
	dkms build -m ach-klinux -v @PACKAGE_VERSION@
	dkms install -m ach-klinux -v @PACKAGE_VERSION@

endif

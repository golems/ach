
if BUILD_DKMS

uninstall-local:
	if test -n "`dkms status ach-klinux/@PACKAGE_VERSION@`"; then \
		dkms remove ach-klinux/@PACKAGE_VERSION@ --all; \
	fi

endif BUILD_DKMS

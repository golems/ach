ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST = CMakeLists.txt spin
# dist_bin_SCRIPTS = achpipe achlog

dist_noinst_SCRIPTS = achtooltest src/run-benchmarks test-achcop


AM_CPPFLAGS = -I$(top_srcdir)/include

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = ach.pc

TESTS = achtest achtooltest test-achcop

include_HEADERS = include/ach.h include/Ach.hpp
noinst_HEADERS = include/achutil.h include/achd.h include/ach_private_posix.h

lib_LTLIBRARIES = libach.la

noinst_LTLIBRARIES = libachutil.la
libachutil_la_SOURCES = src/achutil.c include/achutil.h

bin_PROGRAMS = ach achcat achbench achd achcop achlog
noinst_PROGRAMS = achtest ach-example robusttest

libach_la_SOURCES = src/ach_posix.c src/pipe.c src/dns.c

# This is a libtool version -- CURRENT:REVISION:AGE
# Is /NOT/ major.minor.patch and the relationship is nontrivial
# Does not correspond to the package version
# The cmake versioning needs to be updated when this line changes
libach_la_LDFLAGS = -version-info 3:1:2

ach_SOURCES = src/achtool.c
ach_LDADD = libach.la libachutil.la


achcop_SOURCES = src/achcop.c
achcop_LDADD = libachutil.la

# achpipe_bin_SOURCES = src/achpipe-bin.c
# achpipe_bin_LDADD = libach.la libachutil.la
achlog_SOURCES = src/achlog.c
achlog_LDADD = libach.la libachutil.la

achtest_SOURCES = src/test/achtest.c
achtest_LDADD = libach.la

TESTS += canceltest
noinst_PROGRAMS += canceltest
canceltest_SOURCES = src/test/canceltest.c
canceltest_LDADD = libach.la

TESTS += transfertest
noinst_PROGRAMS += transfertest
transfertest_SOURCES = src/test/transfertest.c
transfertest_LDADD = libach.la

robusttest_SOURCES = src/test/robusttest.c
robusttest_LDADD = libach.la

achcat_SOURCES = src/achcat.c
achcat_LDADD = libach.la libachutil.la

achd_SOURCES = src/achd/achd.c   \
               src/achd/client.c \
               src/achd/io.c \
               src/achd/transport.c
achd_LDADD = libach.la libachutil.la

ach_example_SOURCES = src/ach-example.c
ach_example_LDADD = libach.la -lm

achbench_SOURCES = src/ach-bench.c
achbench_LDADD = libach.la libachutil.la


## C++ ##

lib_LTLIBRARIES += libachcpp.la
libachcpp_la_SOURCES = cpp/achcpp.cpp
libachcpp_la_LDFLAGS = -version-info 0:0:0
libachcpp_la_LIBADD = libach.la

bin_PROGRAMS += cpptest

cpptest_SOURCES = cpp/cpptest.cpp
cpptest_LDADD = libachcpp.la libach.la

## PYTHON ##

if HAVE_PYTHON

python_PYTHON = python/ach.py
pyexec_LTLIBRARIES = ach_py.la
ach_py_la_SOURCES = python/ach_py.c
ach_py_la_LDFLAGS = -avoid-version -module -lach
ach_py_la_CPPFLAGS = $(PYTHON_CPPFLAGS)

endif



## JAVA ##

JSOURCES =                                \
	$(top_srcdir)/java/Lib.java       \
	$(top_srcdir)/java/Status.java    \
	$(top_srcdir)/java/Channel.java   \
	$(top_srcdir)/java/Test.java
EXTRA_DIST += $(JSOURCES) doc/javadoc
if HAVE_JAVAC

BUILT_SOURCES = org_golems_ach_Lib.h


JCLASSES =                                \
	org/golems/ach/Status.class       \
	org/golems/ach/Channel.class      \
	org/golems/ach/Test.class

org/golems/ach/Lib.class: $(JSOURCES)
	$(JAVAC) -d $(top_builddir) $(JSOURCES)

org/golems/ach/Status.class: org/golems/ach/Lib.class
org/golems/ach/Channel.class: org/golems/ach/Lib.class
org/golems/ach/Test.class: org/golems/ach/Lib.class

org_golems_ach_Lib.h: $(top_builddir)/org/golems/ach/Lib.class
	$(JAVAH) org.golems.ach.Lib
	touch $@

org.golems.ach.jar: $(JCLASSES)
	$(JAR) cf $@ $(JCLASSES)


# install the jar file somewhere reasonable
javadir = $(datadir)/java
java_DATA = org.golems.ach.jar

lib_LTLIBRARIES += libachj.la
libachj_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPPFLAGS) $(JNICPPFLAGS)
libachj_la_SOURCES = java/achj.c
libachj_la_LIBADD = libach.la


# Javadocs
doc/.javadoc.stamp: $(JSOURCES)
	javadoc -d doc/javadoc $(JSOURCES)
	touch $@


endif

# All of life's problems are solveable with recursion.

# Automake's uninstall-hook runs after the files are uninstalled.
# DKMS can't removed the ach-klinux module after automake deletes its
# installed sources.  Use a recursive make to force the correct
# ordering.
SUBDIRS = src/klinux/dkms-pre . src/klinux/dkms-post doc


# Linux Module
if BUILD_KBUILD
SUBDIRS += src/klinux
endif

# DKMS
if INSTALL_DKMS
dkmsdir=@prefix@/src/ach-klinux-@PACKAGE_VERSION@
dkms_HEADERS = \
	 include/ach_generic.h \
	 include/ach_impl_generic.h \
	 include/ach_klinux_generic.h \
	 include/ach_klinux.h \
	 include/ach_klinux_kernel.h \
	 include/ach_private_generic.h \
	 include/ach_private_klinux.h
dkms_DATA = src/klinux/dkms/dkms.conf src/klinux/ach_klinux.c src/klinux/dkms/Kbuild

endif

## LOCAL CLEAN ##

clean-local:
	-rm -rf debian/tmp debian/files debian/libach? debian/libach-dev debian/*.log debian/*.substvars texput.log debian/*.debhelper debian/usr/* debian/DEBIAN/* *.so *.so.* org org_golems_ach_Lib.h org.golems.ach.jar $(module_builddir)

distclean-local:
	-rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake .nfs*

maintainer-clean-local:
	-rm -rf  *.tar.gz



include doxygen.am

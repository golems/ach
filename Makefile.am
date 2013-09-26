ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST = CMakeLists.txt spin java/Channel.java java/Lib.java
# dist_bin_SCRIPTS = achpipe achlog

dist_noinst_SCRIPTS = achtooltest src/run-benchmarks test-achcop

SUBDIRS = . doc

AM_CPPFLAGS = -I$(top_srcdir)/include

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = ach.pc

TESTS = achtest achtooltest test-achcop canceltest

include_HEADERS = include/ach.h
noinst_HEADERS = include/achutil.h include/achd.h

lib_LTLIBRARIES = libach.la

noinst_LTLIBRARIES = libachutil.la
libachutil_la_SOURCES = src/achutil.c include/achutil.h

bin_PROGRAMS = ach achcat achbench achd achcop achlog
noinst_PROGRAMS = achtest ach-example canceltest robusttest

libach_la_SOURCES = src/ach.c src/pipe.c

# This is a libtool version -- CURRENT:REVISION:AGE
# Is /NOT/ major.minor.patch and the relationship is nontrivial
# Does not correspond to the package version
# The cmake versioning needs to be updated when this line changes
libach_la_LDFLAGS = -version-info 3:0:2

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

canceltest_SOURCES = src/test/canceltest.c
canceltest_LDADD = libach.la

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

## PYTHON ##

if HAVE_PYTHON

python_PYTHON = python/ach.py
pyexec_LTLIBRARIES = ach_py.la
ach_py_la_SOURCES = python/ach_py.c
ach_py_la_LDFLAGS = -avoid-version -module -lach
ach_py_la_CPPFLAGS = $(PYTHON_CPPFLAGS)

endif



## JAVA ##
if HAVE_JAVAC

BUILT_SOURCES = org_golems_ach_Lib.h

JSOURCES =                                \
	$(top_srcdir)/java/Channel.java   \
	$(top_srcdir)/java/Lib.java

org/golems/ach/Channel.class: $(JSOURCES)
	$(JAVAC) -d $(top_builddir) $(JSOURCES)

org/golems/ach/Lib.class: org/golems/ach/Channel.class

org_golems_ach_Lib.h: $(top_builddir)/org/golems/ach/Lib.class
	javah org.golems.ach.Lib

lib_LTLIBRARIES += libachj.la
libachj_la_CPPFLAGS = $(CPPFLAGS) $(JNICPPFLAGS)
libachj_la_SOURCES = java/achj.c

endif

## LOCAL CLEAN ##

clean-local:
	-rm -rf debian/tmp debian/files debian/libach? debian/libach-dev debian/*.log debian/*.substvars texput.log debian/*.debhelper debian/usr/* debian/DEBIAN/* *.so *.so.* org org_golems_ach_Lib.h

distclean-local:
	-rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake .nfs*

maintainer-clean-local:
	-rm -rf  *.tar.gz



include doxygen.am

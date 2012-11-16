ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST = CMakeLists.txt spin
dist_bin_SCRIPTS = achpipe achlog achtooltest src/run-benchmarks

AM_CPPFLAGS = -I$(top_srcdir)/include

TESTS = achtest achtooltest

include_HEADERS = include/ach.h
noinst_HEADERS = include/achutil.h

lib_LTLIBRARIES = libach.la

if HAVE_RT
bin_PROGRAMS = ach achpipe.bin achcat achbench
noinst_PROGRAMS = achtest achd ach-example
else
bin_PROGRAMS = ach achpipe.bin
noinst_PROGRAMS = achtest
endif

libach_la_SOURCES = src/ach.c src/pipe.c

# This is a libtool version -- CURRENT:REVSION:AGE
# Is /NOT/ major.minor.patch and the relationship is nontrivial
# Does not correspond to the package version
# The cmake versioning needs to be updated when this line changes
libach_la_LDFLAGS = -version-info 1:0:0

ach_SOURCES = src/achtool.c src/achutil.c
ach_LDADD = libach.la

achpipe_bin_SOURCES = src/achpipe-bin.c src/achutil.c
achpipe_bin_LDADD = libach.la

achtest_SOURCES = src/achtest.c
achtest_LDADD = libach.la

if HAVE_RT

achcat_SOURCES = src/achcat.c src/achutil.c
achcat_LDADD = libach.la

achd_SOURCES = src/achd.c src/achutil.c
achd_LDADD = libach.la

ach_example_SOURCES = src/ach-example.c
ach_example_LDADD = libach.la -lm

achbench_SOURCES = src/ach-bench.c src/achutil.c
achbench_LDADD = libach.la

endif

dist_man_MANS = ach.1 achbench.1 achcat.1 achpipe.bin.1 achpipe.1 achlog.1

if HAVE_HELP2MAN
ach.1: src/achtool.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) ach$(EXEEXT)
	help2man -h -h -v -V --no-info -n "manipulate ach channels" ./ach$(EXEEXT) -o ach.1

achbench.1: src/ach-bench.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) achbench$(EXEEXT)
	help2man -h -h -v -V --no-info -n "benchmark and tune ach" ./achbench$(EXEEXT) -o achbench.1

achcat.1: src/achcat.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) achcat$(EXEEXT)
	help2man -h -h -v -V --no-info -n "ach debugging tool" ./achcat$(EXEEXT) -o achcat.1

achpipe.bin.1: src/achpipe-bin.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) achpipe.bin$(EXEEXT)
	help2man -h -h -v -V --no-info -n "send ach messages over streams" ./achpipe.bin$(EXEEXT) -o achpipe.bin.1

achpipe.1: $(top_srcdir)/achpipe
	help2man -h -h -v -V --no-info -n "send ach messages over streams" $(top_srcdir)/achpipe -o achpipe.1

achlog.1: $(top_srcdir)/achlog
	help2man -h -h -v -V --no-info -n "log ach messages" $(top_srcdir)/achlog -o achlog.1
endif

clean-local:
	-rm -rf debian/tmp debian/files debian/libach? debian/libach-dev debian/*.log debian/*.substvars texput.log debian/*.debhelper debian/usr/* debian/DEBIAN/* *.so *.so.*

distclean-local:
	-rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake

maintainer-clean-local:
	-rm -rf *.1 *.tar.gz

include doxygen.am

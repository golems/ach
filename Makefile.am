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
ach.1: ach
	help2man -h -h -v -V --no-info -n "manipulate ach channels" ./ach > ach.1

achbench.1: achbench
	help2man -h -h -v -V --no-info -n "benchmark and tune ach" ./achbench > achbench.1

achcat.1: achcat
	help2man -h -h -v -V --no-info -n "ach debugging tool" ./achcat > achcat.1

achpipe.bin.1: achpipe.bin
	help2man -h -h -v -V --no-info -n "send ach messages over streams" ./achpipe.bin > achpipe.bin.1

achpipe.1: achpipe
	help2man -h -h -v -V --no-info -n "send ach messages over streams" ./achpipe > achpipe.1

achlog.1: achlog
	help2man -h -h -v -V --no-info -n "log ach messages" ./achlog > achlog.1
endif

clean-local:
	-rm -rf debian/tmp debian/files debian/libach? debian/libach-dev debian/*.log debian/*.substvars texput.log debian/*.debhelper debian/usr/* debian/DEBIAN/*

distclean-local:
	-rm -rf *.1

include doxygen.am

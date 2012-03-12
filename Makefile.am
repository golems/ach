ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST = CMakeLists.txt
dist_bin_SCRIPTS = achpipe achlog achtooltest src/run-benchmarks

AM_CPPFLAGS = -I$(top_srcdir)/include

TESTS = achtest achtooltest

include_HEADERS = include/ach.h

lib_LTLIBRARIES = libach.la

if HAVE_RT
bin_PROGRAMS = ach achpipe.bin ach-example achtest achcat ach-bench
else
bin_PROGRAMS = ach achpipe.bin achtest
endif

libach_la_SOURCES = src/ach.c src/ach_stream.c

ach_SOURCES = src/achtool.c
ach_LDADD = libach.la

achpipe_bin_SOURCES = src/achpipe-bin.c
achpipe_bin_LDADD = libach.la

achtest_SOURCES = src/achtest.c
achtest_LDADD = libach.la

if HAVE_RT

achcat_SOURCES = src/achcat.c
achcat_LDADD = libach.la

ach_example_SOURCES = src/ach-example.c
ach_example_LDADD = libach.la

ach_bench_SOURCES = src/ach-bench.c
ach_bench_LDADD = libach.la

endif


clean-local:
	-rm -rf debian/tmp debian/files debian/libach? debian/libach-dev debian/*.log debian/*.substvars texput.log debian/*.debhelper debian/usr/* debian/DEBIAN/*

include doxygen.am

#SUBDIRS = .
EXTRA_DIST = achpipe achlog

INCLUDES = -I$(top_srcdir)/include

include_HEADERS = include/ach.h

lib_LTLIBRARIES = libach.la
libach_la_SOURCES = src/ach.c src/ach_stream.c


bin_PROGRAMS = ach achpipe.bin ach-example achtest achcat ach-bench
ach_SOURCES = src/achtool.c
ach_LDADD = libach.la

achpipe_bin_SOURCES = src/achtool.c
achpipe_bin_LDADD = libach.la

achcat_SOURCES = src/achcat.c
achcat_LDADD = libach.la

achtest_SOURCES = src/achtest.c
achtest_LDADD = libach.la

ach_example_SOURCES = src/ach-example.c
ach_example_LDADD = libach.la

ach_bench_SOURCES = src/ach-bench.c
ach_bench_LDADD = libach.la


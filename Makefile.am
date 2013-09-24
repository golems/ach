ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = -I$(top_srcdir)/include


# include_HEADERS = include/rt4.h

# lib_LTLIBRARIES = librt4kernel.la
# librt4kernel_la_SOURCES = src/kernel.c src/klib.c src/dict.c

# bin_PROGRAMS = rt4
# rt4_SOURCES = src/main.c
# rt4_LDADD = librt4kernel.la

bin_PROGRAMS = bench_lcm

bench_lcm_SOURCES = ipcbench_lcm_timestamp_t.c src/lcm/bench_lcm.c
bench_lcm_LDADD = -llcm


BUILT_SOURCES = ipcbench_lcm_timestamp_t.c ipcbench_lcm_timestamp_t.h

ipcbench_lcm_timestamp_t.h: ipcbench_lcm_timestamp_t.c

ipcbench_lcm_timestamp_t.c: src/lcm/lcm_msg.lcm
	lcm-gen -c $<

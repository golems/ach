ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = -I$(top_srcdir)/include


# include_HEADERS = include/rt4.h

# lib_LTLIBRARIES = librt4kernel.la
# librt4kernel_la_SOURCES = src/kernel.c src/klib.c src/dict.c

# bin_PROGRAMS = rt4
# rt4_SOURCES = src/main.c
# rt4_LDADD = librt4kernel.la

bin_PROGRAMS = ipcbench

ipcbench_SOURCES = src/ipcbench.c

# POSIX methods
ipcbench_SOURCES += src/pipe.c src/mq.c src/tcp.c src/unix.c src/udp.c src/localdgram.c

# Ach
if HAVE_ACH
ipcbench_SOURCES += src/ach.c
LDADD_ACH = -lach
endif

# LCM
if HAVE_LCM
ipcbench_SOURCES += ipcbench_lcm_timestamp_t.c src/lcm.c
LDADD_LCM = -llcm
endif

# Link Libraries
ipcbench_LDADD = $(LDADD_LCM) $(LDADD_ACH)

BUILT_SOURCES = ipcbench_lcm_timestamp_t.c ipcbench_lcm_timestamp_t.h

ipcbench_lcm_timestamp_t.h: ipcbench_lcm_timestamp_t.c

ipcbench_lcm_timestamp_t.c: src/lcm_msg.lcm
	lcm-gen -c $<

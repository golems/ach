PROJECT := ach

VERSION := 0.20110913

SHAREDLIBS := ach

BINFILES := achcat achpipe.bin ach ach-example ach-bench

all: default

# Find the make helper routines
make-common/common.1.mk:
	git submodule init
	git submodule update
include	./make-common/common.1.mk

CFLAGS += -O2 -ansi -DACH_VERSION_STRING=\"$(VERSION)\"
LISPDIR = ./lisp

default: $(LIBFILES) $(BINFILES)  $(BUILDDIR)/achtest
	$(BUILDDIR)/achtest -p 2 -s 4 -n 512

ifneq ($(PLATFORM),Darwin)
CFLAGS += -DHAVE_STRNLEN
endif

bench: $(BUILDDIR)/ach-bench
	$(BUILDDIR)/ach-bench

$(call LINKLIB, ach, ach.o ach_stream.o, pthread rt)

$(call LINKBIN, test_pub, test_pub.c ach.o, pthread rt)
$(call LINKBIN, test_sub, test_sub.c ach.o, pthread rt)
$(call LINKBIN, achcat, achcat.o ach.o, pthread rt)
$(call LINKBIN, achpipe.bin, achpipe.o ach_stream.o ach.o, pthread rt)
$(call LINKBIN, achtest, achtest.o ach_stream.o ach.o, pthread rt)
$(call LINKBIN, ach-example, ach-example.o ach_stream.o ach.o, pthread rt m)
$(call LINKBIN, ach-bench, ach-bench.o ach_stream.o ach.o, pthread rt m)

$(call LINKBIN, ach, ach.o achtool.o, pthread rt)

ach.pyc: ach.py
	./pycompile

clean:
	rm -fv  *.o  test_pub ach.lisp test_sub $(BINFILES) $(LIBFILES) *.deb *.lzma *.pyc
	rm -rf debian doc $(PROJECT)-$(VERSION) .deps build/*

.PHONY: doc example bench

doc: $(INCLUDEDIR)/ach.h
	doxygen


PROJECT := ach

VERSION := 0.20110622

SHAREDLIBS := ach

BINFILES := achcat achpipe.bin ach ach-example

LC_ALL := ascii
LANG := ascii

all: default


# Find the make helper routines
include	$(shell if [ -f /usr/share/make-common/common.1.mk ]; then      \
			echo /usr/share/make-common/common.1.mk;        \
		elif [ -f /usr/local/share/make-common/common.1.mk ]; then \
			echo /usr/local/share/make-common/common.1.mk;  \
		else echo ./common.1.mk;                                \
		fi)

CFLAGS += -O2 --std=gnu99 -fPIC -DACH_VERSION_STRING=\"$(VERSION)\"
LISPDIR = ./lisp

default: $(LIBFILES) $(BINFILES)  achtest
	./achtest -p 2 -s 2 -n 16

ifneq ($(PLATFORM),Darwin)
CFLAGS += -DHAVE_STRNLEN
endif

$(call LINKLIB, ach, ach.o ach_stream.o, pthread rt)

$(call LINKBIN, test_pub, test_pub.c ach.o, pthread rt)
$(call LINKBIN, test_sub, test_sub.c ach.o, pthread rt)
$(call LINKBIN, achcat, achcat.o ach.o, pthread rt)
$(call LINKBIN, achpipe.bin, achpipe.o ach_stream.o ach.o, pthread rt)
$(call LINKBIN, achtest, achtest.o ach_stream.o ach.o, pthread rt)
$(call LINKBIN, ach-example, ach-example.o ach_stream.o ach.o, pthread rt m)

$(call LINKBIN, ach, ach.o achtool.o, pthread rt)

ach.pyc: ach.py
	./pycompile

clean:
	rm -fv  *.o  test_pub ach.lisp test_sub $(BINFILES) $(LIBFILES) *.deb *.lzma *.pyc
	rm -rf debian doc $(PROJECT)-$(VERSION) .deps build/*

.PHONY: doc

doc: $(INCLUDEDIR)/ach.h
	doxygen


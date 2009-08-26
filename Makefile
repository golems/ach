PROJECT := ach

VERSION := 0.20090820

SHAREDLIBS := ach

BINFILES := achcat achpipe.bin ach

#DOXPATH := $(HOME)/prism/public_html/dox

LC_ALL := ascii
LANG := ascii

all: default

include /usr/share/make-common/common.1.mk

default: $(LIBFILES) $(BINFILES) test_sub test_pub ach_stream.o

CFLAGS := -g -Wall -Wextra -Wpointer-arith --std=gnu99 -fPIC -DACH_VERSION_STRING=\"$(VERSION)\" -I$(INCLUDEDIR)

ifneq ($(PLATFORM),Darwin)
CFLAGS += -DHAVE_STRNLEN
endif

$(call LINKLIB, ach, ach.o)

$(call LINKBIN, test_pub, test_pub.c ach.o, pthread rt)
$(call LINKBIN, test_sub, test_sub.c ach.o, pthread rt)
$(call LINKBIN, achcat, achcat.o ach.o, pthread rt)
$(call LINKBIN, achpipe.bin, achpipe.o ach_stream.o ach.o, pthread rt)

$(call LINKBIN, ach, ach.o achtool.o, pthread rt)

clean:
	rm -fv  *.o  test_pub ach.lisp test_sub $(BINFILES) $(LIBFILES) *.deb *.lzma
	rm -rf debian doc $(PROJECT)-$(VERSION)

doc: $(INCLUDEDIR)/ach.h
	doxygen


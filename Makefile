# Project Name
PROJECT := ach

# Project Version
VERSION := 0.20090528

# Library files
LIBFILES := libach.so

DOXPATH := $(HOME)/prism/public_html/dox

default: $(LIBFILES) test_sub test_pub

include /usr/share/make-common/common.1.mk

CFLAGS := -g -Wall -Wextra -Wpointer-arith --std=gnu99 -fPIC

$(call LINKLIB, ach, ach.o)

$(call LINKBIN, test_pub, test_pub.c ach.o, pthread rt)
$(call LINKBIN, test_sub, test_sub.c ach.o, pthread rt)

clean:
	rm -fv  *.o *.so test_pub ach.lisp test_sub

doc: ach.h
	doxygen


# Project Name
PROJECT := ach

# Project Version
VERSION := 20090528

# root to install to for `make install'
PREFIX := /usr/local

# Directory containing C/C++ headers
INCLUDEDIR := .

## Mabe you care about these
cc := gcc
CFLAGS := -g -Wall -Wextra -Wpointer-arith --std=gnu99 -fPIC

# Binary Files
BINFILES :=

# Library files
LIBFILES := libach.so

# Directory of files and links to copy verbatim
#VERBATIMDIR := verbatim

# if you use stow, root of your stow package directory
STOWBASE := /usr/local/stow

# Files to tar up for distribution
DISTFILES := ach.h ach.c Doxyfile Makefile test_pub.c test_sub.c

# Path to copy distribution tarball
DISTPATH := $(PWD)

DISTPATH := $(HOME)/prism/tarballs
DOXPATH := $(HOME)/prism/public_html/dox


include /usr/local/share/make-common/common.1.mk

cc := gcc
CFLAGS := -g -Wall -Wextra -Wpointer-arith --std=gnu99

$(eval $(call LINKLIB, libach.so, ach.o))


ach.o: ach.c $(HEADERS)
	$(cc) $(CFLAGS) -c -o $@ $<

test_pub: test_pub.c ach.o
	$(cc) $(CFLAGS) -o $@ $^ -lpthread -lrt

test_sub: test_sub.c ach.o
	$(cc) $(CFLAGS) -o $@ $^ -lpthread -lrt

clean:
	rm -fv  *.o *.so test_pub ach.lisp

distclean: clean
	rm -rf doc dist

doc: ach.h
	doxygen

ach.lisp: ach.i ach.h
	swig -cffi ach.i



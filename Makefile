## Makefile to ach
##
## Authors: Neil Dantam, Jon Scholz


CFLAGS := -g
PREFIX := /usr/local

STOWBASE := /usr/local/stow
STOWDIR := ach-0.01




STOWPREFIX := $(STOWBASE)/$(STOWDIR)


.PHONY: doc default install clean stow

default: ach.o

install: libach.so
	install --mode=755 libach.so $(PREFIX)/lib
	install --mode=644 ach.h $(PREFIX)/include

stow: libach.so
	mkdir -p $(STOWPREFIX)/lib
	mkdir -p $(STOWPREFIX)/include
	install --mode=755 libach.so $(STOWPREFIX)/lib
	install --mode=644 pach.h $(STOWPREFIX)/include
	cd $(STOWBASE) && stow $(STOWDIR)

doc: ach.h ach.c
	doxygen

clean:
	rm -fv *.o *.so
	rm -rf doc

libach.so: ach.o
	gcc -shared -Wl,-soname,libach.so \
	-o libach.so ach.o

ach.o: ach.c ach.h
	gcc $(CFLAGS) -c -o $@ $<

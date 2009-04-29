
## Edit thise variables to configure installation path
# root to install to for `make install'
PREFIX := /usr/local
# if you use stow, root of your stow package directory
STOWBASE := /usr/local/stow
# if you use stow, a name for this stow package

## Mabe you care about these
cc := gcc
CFLAGS := -g -Wall -Wextra -Wpointer-arith --std=gnu99 -fPIC

## You probably don't care aboute these
VERSION := 0.20090326
PROJECT := ach
PROJVER := $(PROJECT)-$(VERSION)

STOWDIR := $(PROJVER)
STOWPREFIX := $(STOWBASE)/$(STOWDIR)

DISTPATH := $(HOME)/prism/tarballs
DOXPATH := $(HOME)/prism/public_html/dox


HEADERS := ach.h
DISTFILES := ach.h ach.c Doxyfile Makefile test_pub.c


.PHONY: doc default clean stow dist


default: libach.so test_pub test_sub



libach.so: ach.o
	gcc -shared -Wl,-soname,$@ -o $@ $<


ach.o: ach.c $(HEADERS)
	$(cc) $(CFLAGS) -c -o $@ $<

test_pub: test_pub.c ach.o
	$(cc) $(CFLAGS) -o $@ $^ -lpthread -lrt

test_sub: test_sub.c ach.o
	$(cc) $(CFLAGS) -o $@ $^ -lpthread -lrt

clean:
	rm -fv  *.o *.so test_pub

distclean: clean
	rm -rf doc dist

doc: ach.h
	doxygen


stow: libach.so
	mkdir -p $(STOWPREFIX)/include
	mkdir -p $(STOWPREFIX)/lib/
	install --mode 755 libach.so $(STOWPREFIX)/lib
	install --mode 644 ach.h $(STOWPREFIX)/include
	cd $(STOWBASE) && stow $(STOWDIR)



## Developer targets

docul: doc
	cp -Tr doc/html $(DOXPATH)/$(PROJECT)

dist: $(DISTFILES)
	mkdir -p dist/$(PROJVER)
	cp $(DISTFILES) dist/$(PROJVER)
	cd dist &&               \
	tar  --lzma -cvf $(PROJVER).tar.lzma $(PROJVER)
	cp dist/$(PROJVER).tar.lzma $(DISTPATH)

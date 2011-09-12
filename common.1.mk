## Copyright (c) 2009-2011, Georgia Tech Research Corporation
## All rights reserved.
##
## Author(s): Neil T. Dantam <ntd@gatech.edu>
## Georgia Tech Humanoid Robotics Lab
## Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
##
##
## This file is provided under the following "BSD-style" License:
##
##
##   Redistribution and use in source and binary forms, with or
##   without modification, are permitted provided that the following
##   conditions are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##
##   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
##   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
##   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
##   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
##   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
##   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
##   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
##   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
##   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
##   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##   POSSIBILITY OF SUCH DAMAGE.


## FILE: common.1.mk
##
## Some common routines for makefiles.
##
## This is intended for nonrecursive builds.
## See: "Recursive Make Considered Harmful"
##      http://miller.emu.id.au/pmiller/books/rmch/
##
## Almost surely requires gmake.
## You should use a GNU userland.


## USAGE:
# # Your project Makefile
#
# # Set the following variables
#
# # Project Name
# PROJECT := helloworld
#
# # Project Version
# VERSION := 0.0
#
# # Binary Files
# BINFILES := helloexample
#
# # include this file
# include /usr/share/make-common/common.1.mk
#
# # link some stuff
# $(call LINKLIB, foo, foo.o)
# $(call LINKBIN, bar, bar.o bif.o, pthread rt)



#######################
## DEFAULT VARIABLES ##
#######################

## Override these prior to the include as desired

## define compilers

# C Compiler
ifndef cc
cc := gcc
endif

# C++ Compiler
ifndef CC
CC = g++
endif

# Fortran 95 Compiler
ifndef f95
f95 := gfortran
endif

ifndef f77
f77 := gfortran
endif

# Objective C Compiler
ifndef objcc
objcc := gcc
endif

# Static Libary Linker
ifndef ar
ar := ar
endif

# Linker
ifndef ld
ld := gcc
endif

# GNU find
ifndef find
find := $(shell which gfind || which find)
endif

## default directories

ifndef INCLUDEDIR
# ./include if it exists, else .
INCLUDEDIR := $(shell if [ -d ./include ]; then echo ./include; else echo .; fi)
endif

ifndef SRCDIR
# ./src if it exists, else .
SRCDIR := $(shell if [ -d ./src ]; then echo ./src; else echo .; fi)
endif

# must set manually
#ifndef LISPDIR
#LISPDIR := $(shell if [ -d ./lisp ]; then echo ./lisp; else echo .; fi)
#endif

LISPPREFIX ?= $(INSTALLFILES_PREFIX)/share/common-lisp/

ifndef BUILDDIR
# ./build if it exists, else .
BUILDDIR := $(shell if [ -d ./build ]; then echo ./build; else echo .; fi)
endif

ifndef LIBDIR
# ./src if it exists, else .
LIBDIR := $(shell if [ -d $(BUILDDIR)/lib ]; then echo $(BUILDDIR)/lib; else echo $(BUILDDIR); fi)
endif

ifndef FILTER
# ./src if it exists, else .
FILTER := $(shell if [ -f ./filter ]; then echo ". ./filter"; else echo ". /usr/share/make-common/filter"; fi)
endif

ifndef BINDIR
# ./src if it exists, else .
BINDIR := $(shell if [ -d $(BUILDDIR)/bin ]; then echo $(BUILDDIR)/bin; else echo $(BUILDDIR); fi)
endif

ifndef ARCH
ARCH := $(shell uname -m |sed -e 's/x86_64/amd64/' -e 's/armv7l/armel/' -e 's/i686/i386/' )
endif

# directory to search for library includes
LIBDIRS ?= $(LIBDIR)

# Place to install files
PREFIX ?= /usr/local

# Directory to put distribution tarball
DISTPATH ?= .


RSYNCINSTALL ?= rsync --delete -r -f "$(FILTER)"

## default compiler flags

# C
ifndef CFLAGS
#CFLAGS := -g  -Wc++-compat -Wall -Wextra -Wconversion -Wpointer-arith -Wfloat-equal -Wshadow -Wframe-larger-than=2048 -Wlarger-than=104857600 -Wwrite-strings -Wlogical-op -I$(INCLUDEDIR)
CFLAGS := -g  -Wc++-compat -Wall -Wextra -Wconversion -Wpointer-arith -Wfloat-equal -Wshadow -Wwrite-strings -I$(INCLUDEDIR)
endif
# AMD64 requires PIC for shared libs (so just use it everywhere)
ifeq ($(ARCH),amd64)
CFLAGS += -fPIC
endif

# C++
ifndef CPPFLAGS
#CPPFLAGS := -g  -Wall -Wextra -Wconversion -Wpointer-arith -Wfloat-equal -Wshadow -Wframe-larger-than=2048 -Wlarger-than=104857600 -Wwrite-strings -Wlogical-op -I$(INCLUDEDIR)
CPPFLAGS := -g  -Wall -Wextra -Wconversion -Wpointer-arith -Wfloat-equal -Wshadow -Wwrite-strings -I$(INCLUDEDIR)
endif
ifeq ($(ARCH),amd64)
CPPFLAGS += -fPIC
endif

# Objective-C
OBJCFLAGS ?= $(CFLAGS)


# Fortran
FFLAGS ?= -g -fimplicit-none -J$(INCLUDEDIR)

# Linker
LDFLAGS ?= -shared

## Debian Package Flags

# directory for building deb package
DEBDIR ?= debian

# directory to put .deb files
DEBDISTDIR ?= .

ifndef DEBCONTROLBASE
DEBCONTROL := $(shell if [ -f control ]; then echo control; fi)
endif

DEBPKGVERSION ?= 1

DEBPREFIX ?= /usr

## default source files

ifndef SRCFILES
SRCFILES := $(shell $(find) .  \( -type d \( -name .svn -o -name .git \)  -prune \)  -o -regex '.*\.\(c\|cpp\|f90|f95\)' -print)
# Maybe svn list -R | egrep '^.*\.(c|cpp|f95)$'
# It would be nicer (40x) on a cold disk cache, but slower (4x) otherwise
# Would also fail if files are not yet svn add'ed
endif

VERBATIMDIR ?= verbatim

# if you use stow, root of your stow package directory
STOWBASE ?= $(PREFIX)/stow

DEPDIR ?= .deps

comma := ,

PROJVER := $(PROJECT)-$(VERSION)

STOWDIR := $(PROJVER)
STOWPREFIX := $(STOWBASE)/$(STOWDIR)


## Dependency Files
## (gcc will generate dependency info for C and C++ files and spit out make rules)
## We use one dep file for each source file to minimize dependency regenerations
## as recommended by the GNU Make manual.
DEPFILES := $(addprefix $(DEPDIR)/,$(addsuffix .d, $(filter %.c %.cpp %.cc %.m, $(patsubst $(SRCDIR)/%, %, $(SRCFILES)))))

## Use the version control system to figure which files are canonical
## TODO: support git
ifndef DISTFILES
DISTFILES = $(shell if [ -d ./.svn ]; then svn list --recursive; fi )
endif

#######################
## PORTABILITY TESTS ##
#######################
# Not much here yet

# Fetch platform name if we don't have it from elsewhere (we probably don't)
ifndef PLATFORM
PLATFORM := $(shell uname -s)
OS := $(shell uname -o 2>/dev/null)
endif

# (courtesy of Jon Olson)
## OS X uses a different library extension, play nice
ifeq ($(PLATFORM),Darwin)
# somebody who can actually stomach "The Apple Way" should test this...
SHARED_LIB_SUFFIX := .dylib
LDFLAGS := -lc
else
SHARED_LIB_SUFFIX := .so
endif

##############################
## MAKE GNULIB GO IF NEEDED ##
##############################

# The world would be a much better place of BSD had won. Fuck GNU.

ifeq (,$(findstring GNU,$(OS)))
# System isn't GNU, set GNU deps
ifdef GNULIB_MODULES
GNULIB_MODULES += progname
GNUDEPS := gnulib/gllib/libgnu.a gnulib/gnulib.h
GNU_CFLAGS := -Ignulib/gllib -include gnulib/gnulib.h
GNU_LDFLAGS := -Lgnulib/gllib -lgnu
endif

gnulib/gllib/libgnu.a:
	rm -rf gnulib
	gnulib-tool --create-testdir --dir=gnulib $(GNULIB_MODULES)
	cd gnulib && ./configure
	make -C gnulib

gnulib/gnulib.h:
	$(foreach module,$(GNULIB_MODULES),$(shell gnulib-tool --extract-include-directive $(module) >>gnulib/gnulib.h))

LD_DYNAMIC := -dynamic
LD_STATIC := -static
else
LD_DYNAMIC := -Bdynamic
LD_STATIC := -Bstatic
endif

#####################
## FIXUP VARIABLES ##
#####################

LIBFILES := $(addprefix $(LIBDIR)/lib, $(addsuffix $(SHARED_LIB_SUFFIX), $(SHAREDLIBS)))
BINFILES := $(addprefix $(BINDIR)/, $(BINFILES))

###############
## FUNCTIONS ##
###############

## Convenience method for linking shared libraries
## For some reason, automatic variables don't work here...
## call with  $(call LINKLIB, name_of_lib, list of object files)
## ie with $(call LINKLIB, frob, foo.o bar.o) # gives libfrob.so
define LINKLIB1
$(LIBDIR)/lib$(strip $(1))$(SHARED_LIB_SUFFIX): $(GNUDEPS) $(2)
ifeq ($(PLATFORM),Darwin)
	$(cc) -dynamiclib $(GNU_LDFLAGS) $(LDFLAGS) -o $(LIBDIR)/lib$(strip $(1))$(SHARED_LIB_SUFFIX) $(2) $(addprefix -l,$(3))
else
	@echo [ld] $(1)$(SHARED_LIB_SUFFIX)
	@$(ld) $(GNU_LDFLAGS) $(LDFLAGS) -o $(LIBDIR)/lib$(strip $(1))$(SHARED_LIB_SUFFIX)  $(2) $(addprefix -l,$(3))
endif
endef

# this def does the eval so the caller doesn't have to
define LINKLIB
$(eval $(call LINKLIB1, $1, $(addprefix $(BUILDDIR)/, $2), $3))
endef

## Convenience method for linking binaries
## call with $(call LINKBIN, name_of_binary, object files, shared libs, static libs)
## ie  $(call LINKBIN frob, foo.o, bar, bif)
define LINKBIN1
$(BINDIR)/$(strip $(1)): $(GNUDEPS) $(2)  $(addsuffix .a, $(addprefix lib, $(4)))
	@echo [ld] $(1)
	@$(cc) $(CFLAGS) -o $(BINDIR)/$(strip $(1)) $(2) \
	  $(addprefix -L, $(LIBDIRS))  \
	  $(if $(strip $(4)), -Wl$(comma)$(LD_STATIC)) $(addprefix -l, $(strip $(4)))  \
	  $(if $(strip $(3)), -Wl$(comma)$(LD_DYNAMIC)) $(addprefix -l, $(3)) $(foo) \
	  $(GNU_LDFLAGS)
endef

# this def does the eval so the caller doesn't have to
define LINKBIN
$(eval $(call LINKBIN1, $1, $(addprefix $(BUILDDIR)/, $2), $3, $4))
endef



#############
## TARGETS ##
#############

## Target to print out the defined variables
env:
	@echo PROJECT: $(PROJECT)
	@echo VERSION: $(VERSION)
	@echo PREFIX: $(PREFIX)
	@echo SRCDIR: $(SRCDIR)
	@echo LISPDIR: $(LISPDIR)
	@echo INCLUDEDIR: $(INCLUDEDIR)
	@echo BUILDDIR: $(BUILDDIR)
	@echo LIBDIR: $(LIBDIR)
	@echo BINDIR: $(BINDIR)
	@echo PLATFORM: $(PLATFORM)
	@echo cc: $(cc)
	@echo CC: $(CC)
	@echo f95: $(f95)
	@echo objcc: $(objcc)
	@echo ld: $(ld)
	@echo ar: $(ar)
	@echo CFLAGS: $(CFLAGS)
	@echo CPPFLAGS: $(CPPFLAGS)
	@echo OBJCFLAGS: $(OBJCFLAGS)
	@echo FFLAGS: $(FFLAGS)
	@echo LDFLAGS: $(LDFLAGS)
	@echo SRCFILES: $(SRCFILES)
	@echo OBJFILES: $(OBJFILES)
	@echo DEPFILES: $(DEPFILES)
	@echo LIBFILES: $(LIBFILES)
	@echo BINFILES: $(BINFILES)
	@echo SHARED_LIB_SUFFIX: $(SHARED_LIB_SUFFIX)
	@echo DEBDIR: $(DEBDIR)
	@echo DEBDISTDIR: $(DEBDISTDIR)
	@echo DEBCONTROLBASE: $(DEBCONTROLBASE)
	@echo DEBPKGVERSION: $(DEBPKGVERSION)
	@echo DISTPATH: $(DISTPATH)
	@echo FILTER: $(FILTER)
	@echo RSYNCINSTALL: $(RSYNCINSTALL)
	@echo ARCH: $(ARCH)




TERM_GREEN="\033[0;32m"
TERM_NO_COLOR="\033[0m"
TERM_LIGHT_GREEN="\033[1;32m"


installfiles:
	@echo $(TERM_LIGHT_GREEN)'* INSTALLING TO $(INSTALLFILES_PREFIX) *'$(TERM_NO_COLOR)
	@echo $(TERM_LIGHT_GREEN)'* INSTALLING BINARIES *'$(TERM_NO_COLOR)
	@if test -n "$(BINFILES)"; then \
		mkdir -vp $(INSTALLFILES_PREFIX)/bin; \
		install -v --mode 755 $(BINFILES) $(INSTALLFILES_PREFIX)/bin; \
	fi
	@echo $(TERM_LIGHT_GREEN)'* INSTALLING LIBS *'$(TERM_NO_COLOR)
	@if test -n "$(LIBFILES)"; then \
		mkdir -vp $(INSTALLFILES_PREFIX)/lib; \
		install -v --mode 755 $(LIBFILES) $(INSTALLFILES_PREFIX)/lib; \
	fi
	@echo $(TERM_LIGHT_GREEN)'* INSTALLING HEADERS *'$(TERM_NO_COLOR)
	@if test -d "$(INCLUDEDIR)"; then \
		mkdir -vp $(INSTALLFILES_PREFIX)/include; \
		(cd $(INCLUDEDIR) && tar cf -  `$(find) -regex '.*\.\(h\|hpp\|mod\)'` )  \
		  | (cd $(INSTALLFILES_PREFIX)/include && tar xvf - ) \
	fi
	@if test -d "$(LISPDIR)"; then \
		echo $(TERM_LIGHT_GREEN)'* INSTALLING LISP FILES *'$(TERM_NO_COLOR); \
		mkdir -pv $(LISPPREFIX)/source/$(PROJECT); \
		mkdir -pv $(LISPPREFIX)/systems; \
		(cd $(LISPDIR) && tar cf -  `$(find) -regex '.*\.\(lisp\|asd\)'` )  \
		  | (cd $(LISPPREFIX)/source/$(PROJECT) && tar xvf - ) 	; \
		(cd $(LISPPREFIX)/systems && $(find) ../source/$(PROJECT) -name '*.asd' \
			-exec ln -fvs '{}' ';' ); \
	fi
	@echo $(TERM_LIGHT_GREEN)'* INSTALLING VERBATIM *'$(TERM_NO_COLOR)
	@if test -d "$(VERBATIMDIR)"; then \
	    mkdir -vp $(INSTALLFILES_PREFIX); \
	    ( cd $(VERBATIMDIR) && tar -hcf - \
	     `$(find) . '!' \( -type d  \( -name .svn -o -name .git \) -prune \) -type f -o -type l`) |\
	   (cd $(INSTALLFILES_PREFIX) && tar xvf - ) \
	fi
	@if test -d ".svn"; then \
		echo $(TERM_LIGHT_GREEN)'* NOTING SVN REVISION *'$(TERM_NO_COLOR); \
		mkdir -pv $(INSTALLFILES_PREFIX)/share/$(PROJECT);             \
		svn info > $(INSTALLFILES_PREFIX)/share/$(PROJECT)/svn-info;   \
	fi
deb: INSTALLFILES_PREFIX := $(DEBDIR)$(DEBPREFIX)
deb: installfiles
	@echo $(TERM_LIGHT_GREEN)'* Making DEB *'$(TERM_NO_COLOR)
	if [ -d "$(ETCDIR)" ] ; then $(RSYNCINSTALL) "$(ETCDIR)" $(DEBDIR); fi
	mkdir -pv $(DEBDIR)/DEBIAN
	echo Package: $(PROJECT) > $(DEBDIR)/DEBIAN/control
	echo Version: $(VERSION)-$(DEBPKGVERSION) >> $(DEBDIR)/DEBIAN/control
	echo Architecture: $(ARCH) >> $(DEBDIR)/DEBIAN/control
	if [ -f "$(DEBCONTROL)" ] ; then cat $(DEBCONTROL) >> $(DEBDIR)/DEBIAN/control; \
	else \
	  echo Maintainer: unkown >> $(DEBDIR)/DEBIAN/control; \
	  echo Description: none >> $(DEBDIR)/DEBIAN/control; \
	fi
	fakeroot dpkg-deb --build $(DEBDIR) $(DEBDISTDIR)/$(PROJECT)_$(VERSION)-$(DEBPKGVERSION).deb

debinstall: DEBTEMP := $(shell tempfile)
debinstall: DEBFILE := $(DEBDISTDIR)/$(PROJECT)_$(VERSION)-$(DEBPKGVERSION).deb
debinstall: deb
	sudo dpkg -i $(DEBFILE)
	@if [ -n "$(RHOST)" ]; then scp $(DEBFILE) $(RHOST):/tmp && ssh $(RHOST) "sudo dpkg -i /tmp/$(DEBFILE); rm /tmp/$(DEBFILE)"; fi

dpkgi: DEBTEMP := $(shell tempfile)
dpkgi:
	cp $(DEBDISTDIR)/$(PROJECT)_$(VERSION)-$(DEBPKGVERSION).deb $(DEBTEMP)
	sudo dpkg -i $(DEBTEMP)
	rm $(DEBTEMP)

#stow: INSTALLFILES_PREFIX := $(STOWPREFIX)
#stow: installfiles
#	cd $(STOWBASE) && stow $(STOWDIR)

#install: INSTALLFILES_PREFIX := $(PREFIX)
#install: installfiles


## Developer targets

docul: doc
	if test -d "$(DOXPATH)"; then \
	  cp -Tr doc/html $(DOXPATH)/$(PROJECT); \
	elif test -n "$(DOXRSYNCSSH)"; then \
	  cd doc/html && rsync --delete -rve ssh . $(DOXRSYNCSSH)/$(PROJECT);\
	fi

dist:
	rm -rf $(BUILDDIR)/$(PROJVER)
	mkdir -vp $(BUILDDIR)/$(PROJVER)
	$(foreach file, $(DISTFILES), \
	  if test -d $(file); \
	    then mkdir -vp $(BUILDDIR)/$(PROJVER)/$(file); \
	  else ln -v $(file) $(BUILDDIR)/$(PROJVER)/$(file);\
	  fi && ) true
	cd $(BUILDDIR) && tar -cjvf $(PROJVER).tar.bz2 $(PROJVER)
	if test -n "$(DISTSCPPATH)"; then scp $(BUILDDIR)/$(PROJVER).tar.bz2 $(DISTSCPPATH); fi



####################
## IMPLICIT RULES ##
####################

## These create targets to build C, C++, Fortran, and Objective C

# C
$(BUILDDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -vp $(dir $(@))
	@echo [cc] $<
	@$(cc) $(GNU_CFLAGS) $(CFLAGS) -c $< -o $@

# C++
$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -vp $(dir $(@))
	@echo [CC] $<
	@$(CC) $(GNU_CFLAGS) $(CPPFLAGS) -c $< -o $@

# C++ again
$(BUILDDIR)/%.o: $(SRCDIR)/%.cc
	@mkdir -vp $(dir $(@))
	@echo [CC] $<
	@$(CC) $(GNU_CFLAGS) $(CPPFLAGS) -c $< -o $@

# Fortran 95
$(BUILDDIR)/%.o: $(SRCDIR)/%.f95
	@mkdir -vp $(dir $(@))
	@echo [fc] $<
	@$(f95) $(FFLAGS) -c $< -o $@

$(BUILDDIR)/%.o: $(SRCDIR)/%.f90
	@mkdir -vp $(dir $(@))
	@echo [fc] $<
	@$(f95) $(FFLAGS) -c $< -o $@

$(BUILDDIR)/%.o: $(SRCDIR)/%.f
	@mkdir -vp $(dir $(@))
	@echo [fc] $<
	@$(f77) $(FFLAGS) -c $< -o $@

# Objective C
$(BUILDDIR)/%.o: $(SRCDIR)/%.m
	@mkdir -vp $(dir $(@))
	$(objcc) $(GNU_CFLAGS) $(OBJCFLAGS) -c $< -o $@


## Rules to generate dependecy info
## Hopefully gfortran will do this too, soon

$(DEPDIR)/%.c.d: $(SRCDIR)/%.c
	@mkdir -pv $(dir $@)
	@/bin/echo -n ./$(dir $<) | sed -s 's!$(SRCDIR)!$(BUILDDIR)!'  > $@
	@echo [DEP] $<
	@$(cc) $(CFLAGS) -MM  $< >> $@ || rm $@

$(DEPDIR)/%.cpp.d: $(SRCDIR)/%.cpp
	@mkdir -pv $(dir $@)
	@/bin/echo -n ./$(dir $<) | sed -s 's!$(SRCDIR)!$(BUILDDIR)!'  > $@
	@echo [DEP] $<
	@$(CC) $(CPPFLAGS) -MM  $< >> $@ || rm $@

$(DEPDIR)/%.cc.d: $(SRCDIR)/%.cc
	@mkdir -pv $(dir $@)
	@/bin/echo -n $(dir $<)  > $@
	@echo [DEP] $<
	@$(CC) $(CPPFLAGS) -MM  $< >> $@ || rm $@

########################
## DEPENDENCY INCLUDE ##
########################
## Include the auto-generated dependencies
-include $(DEPFILES)


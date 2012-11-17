ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST = CMakeLists.txt spin doc/ach.html
dist_bin_SCRIPTS = achpipe achlog achtooltest src/run-benchmarks

AM_CPPFLAGS = -I$(top_srcdir)/include

TESTS = achtest achtooltest

include_HEADERS = include/ach.h
noinst_HEADERS = include/achutil.h

lib_LTLIBRARIES = libach.la

if HAVE_RT
bin_PROGRAMS = ach achpipe.bin achcat achbench
noinst_PROGRAMS = achtest achd ach-example
else
bin_PROGRAMS = ach achpipe.bin
noinst_PROGRAMS = achtest
endif

libach_la_SOURCES = src/ach.c src/pipe.c

# This is a libtool version -- CURRENT:REVSION:AGE
# Is /NOT/ major.minor.patch and the relationship is nontrivial
# Does not correspond to the package version
# The cmake versioning needs to be updated when this line changes
libach_la_LDFLAGS = -version-info 1:0:0

ach_SOURCES = src/achtool.c src/achutil.c
ach_LDADD = libach.la

achpipe_bin_SOURCES = src/achpipe-bin.c src/achutil.c
achpipe_bin_LDADD = libach.la

achtest_SOURCES = src/achtest.c
achtest_LDADD = libach.la

if HAVE_RT

achcat_SOURCES = src/achcat.c src/achutil.c
achcat_LDADD = libach.la

achd_SOURCES = src/achd.c src/achutil.c
achd_LDADD = libach.la

ach_example_SOURCES = src/ach-example.c
ach_example_LDADD = libach.la -lm

achbench_SOURCES = src/ach-bench.c src/achutil.c
achbench_LDADD = libach.la

endif

dist_man_MANS = doc/ach.1 doc/achbench.1 doc/achcat.1 doc/achpipe.bin.1 doc/achpipe.1 doc/achlog.1 doc/achd.1
dist_doc_DATA = doc/ach.html doc/achbench.html doc/achcat.html doc/achpipe.bin.html doc/achpipe.html doc/achlog.html doc/achd.html

if HAVE_HELP2MAN
doc/ach.1: src/achtool.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) ach$(EXEEXT)
	mkdir -p doc
	help2man -h -h -v -V --no-info -n "manipulate ach channels" ./ach$(EXEEXT) -o $@

doc/achbench.1: src/ach-bench.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) achbench$(EXEEXT)
	mkdir -p doc
	help2man -h -h -v -V --no-info -n "benchmark and tune ach" ./achbench$(EXEEXT) -o $@

doc/achcat.1: src/achcat.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) achcat$(EXEEXT)
	mkdir -p doc
	help2man -h -h -v -V --no-info -n "ach debugging tool" ./achcat$(EXEEXT) -o $@

doc/achpipe.bin.1: src/achpipe-bin.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) achpipe.bin$(EXEEXT)
	mkdir -p doc
	help2man -h -h -v -V --no-info -n "send ach messages over streams" ./achpipe.bin$(EXEEXT) -o $@

doc/achd.1: src/achd.c $(top_srcdir)/configure.ac
	$(MAKE) $(AM_MAKEFLAGS) achpipe.bin$(EXEEXT)
	mkdir -p doc
	help2man -h -h -v -V --no-info -n "send ach messages over streams" ./achd$(EXEEXT) -o $@

doc/achpipe.1: $(top_srcdir)/achpipe
	mkdir -p doc
	help2man -h -h -v -V --no-info -n "send ach messages over streams" $(top_srcdir)/achpipe -o $@

doc/achlog.1: $(top_srcdir)/achlog
	mkdir -p doc
	help2man -h -h -v -V --no-info -n "log ach messages" $(top_srcdir)/achlog -o $@

if HAVE_MAN2HTML
.1.html:
	man2html -r $< | sed -e '1d;2d;8d;s/@\|&amp;/{at}/g' > $@
endif

endif

clean-local:
	-rm -rf debian/tmp debian/files debian/libach? debian/libach-dev debian/*.log debian/*.substvars texput.log debian/*.debhelper debian/usr/* debian/DEBIAN/* *.so *.so.*

distclean-local:
	-rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake

maintainer-clean-local:
	-rm -rf doc/*.1 doc/*.html *.tar.gz

include doxygen.am

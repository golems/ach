dist_man_MANS = ach.1 achbench.1 achcat.1 achpipe.bin.1 achpipe.1 achlog.1 achd.1
dist_doc_DATA = ach.html achbench.html achcat.html achpipe.bin.html achpipe.html achlog.html achd.html

if HAVE_HELP2MAN
ach.1: $(top_srcdir)/src/achtool.c $(top_srcdir)/configure.ac
	$(HELP2MAN) -h -h -v -V --no-info -n "manipulate ach channels" $(top_builddir)/ach$(EXEEXT) -o $@

achbench.1: $(top_srcdir)/src/ach-bench.c $(top_srcdir)/configure.ac
	$(HELP2MAN) -h -h -v -V --no-info -n "benchmark and tune ach" $(top_builddir)/achbench$(EXEEXT) -o $@

achcat.1: $(top_srcdir)/src/achcat.c $(top_srcdir)/configure.ac
	$(HELP2MAN) -h -h -v -V --no-info -n "ach debugging tool" $(top_builddir)/achcat$(EXEEXT) -o $@

achpipe.bin.1: $(top_srcdir)/src/achpipe-bin.c $(top_srcdir)/configure.ac
	$(HELP2MAN) -h -h -v -V --no-info -n "send ach messages over streams" $(top_builddir)/achpipe.bin$(EXEEXT) -o $@

achd.1: $(top_srcdir)/src/achd.c $(top_srcdir)/configure.ac
	$(HELP2MAN) -h -\? -v -V --no-info -n "send ach messages over streams" $(top_builddir)/achd$(EXEEXT) -o $@

achpipe.1: $(top_srcdir)/achpipe
	$(HELP2MAN) -h -h -v -V --no-info -n "send ach messages over streams" $(top_srcdir)/achpipe -o $@

achlog.1: $(top_srcdir)/achlog
	$(HELP2MAN) -h -h -v -V --no-info -n "log ach messages" $(top_srcdir)/achlog -o $@

if HAVE_MAN2HTML
.1.html:
	$(MAN2HTML) -r $< | sed -e '1d;2d;8d;s/@\|&amp;/{at}/g' > $@
endif

endif


maintainer-clean-local:
	-rm -rf *.1 *.html

EXTRA_DIST = achpipe.bin.inc achd.inc

if HAVE_HELP2MAN
dist_man_MANS = ach.1 achbench.1 achcat.1 achpipe.bin.1 achpipe.1 achlog.1 achd.1
ach.1: $(top_srcdir)/src/achtool.c $(top_srcdir)/configure.ac
	$(HELP2MAN) -h -h -v -V --no-info -n "manipulate ach channels" $(top_builddir)/ach$(EXEEXT) -o $@

achbench.1: $(top_srcdir)/src/ach-bench.c $(top_srcdir)/configure.ac
	$(HELP2MAN) -h -h -v -V --no-info -n "benchmark and tune ach" $(top_builddir)/achbench$(EXEEXT) -o $@

achcat.1: $(top_srcdir)/src/achcat.c $(top_srcdir)/configure.ac
	$(HELP2MAN) -h -h -v -V --no-info -n "ach debugging tool" $(top_builddir)/achcat$(EXEEXT) -o $@

achpipe.bin.1: $(top_srcdir)/src/achpipe-bin.c $(top_srcdir)/configure.ac achpipe.bin.inc
	$(HELP2MAN) -h -h -v -V --no-info -i achpipe.bin.inc -n "send ach messages over streams" $(top_builddir)/achpipe.bin$(EXEEXT) -o $@

achd.1: $(top_srcdir)/src/achd/achd.c $(top_srcdir)/configure.ac achd.inc
	$(HELP2MAN) -h -\? -v -V -i achd.inc --no-info -n "send ach messages over streams" $(top_builddir)/achd$(EXEEXT) -o $@

achpipe.1: $(top_srcdir)/achpipe
	$(HELP2MAN) -h -h -v -V --no-info -n "send ach messages over streams" $(top_srcdir)/achpipe -o $@

achlog.1: $(top_srcdir)/achlog
	$(HELP2MAN) -h -h -v -V --no-info -n "log ach messages" $(top_srcdir)/achlog -o $@

if HAVE_MAN2HTML
MAN_HTML = ach.html achbench.html achcat.html achpipe.bin.html achpipe.html achlog.html achd.html
.1.html:
	$(MAN2HTML) -r $< | sed -e '1d;2d;8d;s/@\|&amp;/{at}/g' > $@
endif

endif


if HAVE_DB2HTML
DOCBOOK_HTML =  ach-manual.html
ach-manual.html: ach-manual.xml
	db2html -u -o . $<
endif


dist_doc_DATA = $(DOCBOOK_HTML) $(MAN_HTML)


maintainer-clean-local:
	-rm -rf *.1 *.html

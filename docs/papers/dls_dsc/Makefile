%.html: %.rst
	rst2html $< $@

# Various references for generating PDF from TeX and ReST.
#   http://docutils.sourceforge.net/docs/user/latex.html
# Options for tex->pdf are:
#  1. pdflatex
#  2. latex + dvipdfmx, need to add dvipdfmx to document options
#  3. laxex + dvips + ps2pdf, allows PS specials to be interpreted
%.pdf: %.tex
	pdflatex $<
	pdflatex $<
	rm -f $*.aux $*.log $*.out $*.toc

%.tex: %.rst
	rst2latex $< $@

default: dsc_daemon.html dsc_daemon.pdf

clean:
	rm -f *.html *.pdf

.PHONY: default clean

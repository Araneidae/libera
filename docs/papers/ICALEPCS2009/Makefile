CITE_GREP = There were undefined references.
REF_GREP = Rerun to get cross-references right.

%.pstex_t: %.fig
	fig2dev -L pstex $*.fig $*.pstex
	fig2dev -L pstex_t -F -p $*.pstex $*.fig $*.pstex_t

%.dvi: %.tex
	latex $*
	if grep -q '$(CITE_GREP)\|$(REF_GREP)' $*.log; then latex $*; fi

%.eps: %.png
	bmeps -c $< $@

%.ps: %.dvi
	dvips -o $@ $<

%.pdf: %.dvi
	dvips -j0 -G0 -o - -Ppdf $* | \
        ps2pdf13 -dPDFSETTINGS=/prepress - $@
#        ps2pdf13 -dPDFSETTINGS=/prepress -dAntiAliasColorImages=true - $@


FIGURES = system-overview buttons

Libera.dvi:  $(FIGURES:=.tex)

default: Libera.dvi

pdf: Libera.pdf

ps: Libera.ps


clean:
	rm -f *.log *.aux

clean-targets:
	rm -f *.dvi *.pdf *.ps

clean-all: clean clean-targets

RERUN_GREP='Rerun to get (citations correct|cross-references right)'

%.dvi: %.tex
	latex $*
	if grep -E $(RERUN_GREP) $*.log; then latex $*; fi

%.ps: %.dvi
	dvips -o $@ $<

%.pdf: %.dvi
	echo $($*_OPT)
	dvips -j0 -G0 -o - -Ppdf $* | ps2pdf - $@

# %.pdf: %.tex
# 	pdflatex $<

DOCUMENTS = requirements libera

default: ps pdf

dvi: $(patsubst %, %.dvi, $(DOCUMENTS))

ps: $(patsubst %, %.ps, $(DOCUMENTS))

pdf: $(patsubst %, %.pdf, $(DOCUMENTS))

.PHONY: clean
clean:
	rm -f *.ps *.dvi *.pdf *.aux *.log


# Target required for cooperation with EPICS make system.
install:  default


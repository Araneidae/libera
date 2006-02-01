%.dvi: %.tex
	latex $<

%.ps: %.dvi
	dvips -o $@ $<

%.pdf: %.tex
	pdflatex $<

default: ps pdf

dvi: requirements.dvi

ps: requirements.ps

pdf: requirements.pdf

.PHONY: clean
clean:
	rm -f *.ps *.dvi *.pdf *.aux *.log


# Target required for cooperation with EPICS make system.
install:  default


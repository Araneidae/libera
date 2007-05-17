%.dvi: %.tex
	latex $<

%.ps: %.dvi
	dvips -o $@ $<

%.pdf: %.tex
	pdflatex $<

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


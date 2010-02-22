ASCIIDOC = /home/mga83/local/bin/asciidoc

%.html: %.txt
	$(ASCIIDOC) $<

%.xml: %.txt
	$(ASCIIDOC) -b docbook $<

%.pdf: %.xml
	docbook2pdf $<


DOCUMENTS = libera CHANGES

default: check-pv-docs $(patsubst %, %.html, $(DOCUMENTS))

clean:
	rm -f *.html

check-pv-docs:
	./check-pv-docs

.PHONY: clean check-pv-docs

ASCIIDOC = /home/mga83/local/bin/asciidoc

%.html: %.txt
	$(ASCIIDOC) $<

%.xml: %.txt
	$(ASCIIDOC) -b docbook $<

%.pdf: %.xml
	docbook2pdf $<


DOCUMENTS = libera CHANGES

default: $(patsubst %, %.html, $(DOCUMENTS))


.PHONY: clean check-pv-docs
clean:
	rm -f *.html

check-pv-docs:
	./check-pv-docs

libera.html: check-pv-docs

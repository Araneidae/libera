ASCIIDOC = /home/mga83/local/bin/asciidoc

%.html: %.txt
	$(ASCIIDOC) $<


DOCUMENTS = libera CHANGES

default: $(patsubst %, %.html, $(DOCUMENTS))


.PHONY: clean
clean:
	rm -f *.html


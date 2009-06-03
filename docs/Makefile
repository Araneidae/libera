ASCIIDOC = /home/mga83/local/bin/asciidoc

%.html: %.txt
	$(ASCIIDOC) $<



default: $(patsubst %.txt, %.html, $(wildcard *.txt))


.PHONY: clean
clean:
	rm -f *.html


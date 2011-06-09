RST2HTML = rst2html

%.html: %.rst
	$(RST2HTML) $< >$@


DOCUMENTS = libera CHANGES process

default: check-pv-docs $(patsubst %, %.html, $(DOCUMENTS))

clean:
	rm -f *.html

check-pv-docs:
	./check-pv-docs

.PHONY: clean check-pv-docs

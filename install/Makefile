TOP = ..
include $(TOP)/configure/RELEASE

T_A = linux-arm
BIN_DIR = $(TOP)/bin/$(T_A)


# The install target in this makefile will be run after successful completion
# of the rest of the project makefile.  We patch up the installation ready
# for deployment to Libera.
install: locations $(BIN_DIR)/runioc

clean:
	rm -f locations

# The list of possible installation locations is derived from CONFIG.
LOC_SED_COMMAND = '/^([[:alpha:]][[:alnum:]])_IOC_.*$$/{s//\1/;p;}' 
locations: CONFIG
	sed -rn $(LOC_SED_COMMAND) $< | sort | uniq >$@

$(BIN_DIR)/runioc: runioc
	cp $< $@

TOP = ..
include $(TOP)/configure/CONFIG

# The install target in this makefile will be run after successful completion
# of the rest of the project makefile.  We patch up the installation ready
# for deployment to Libera.
install $(CROSS_COMPILER_TARGET_ARCHS): locations
realclean archclean: clean

clean:
	rm -f locations

# The list of possible installation locations is derived from CONFIG.
LOC_SED_COMMAND = '/^([[:alpha:]][[:alnum:]])_IOC_.*$$/{s//\1/;p;}' 
locations: CONFIG
	sed -rn $(LOC_SED_COMMAND) $< | sort | uniq >$@


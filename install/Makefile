TOP = ..
include $(TOP)/configure/RELEASE

T_A = linux-arm
BIN_DIR = $(TOP)/bin/$(T_A)


# The install target in this makefile will be run after successful completion
# of the rest of the project makefile.  We patch up the installation ready
# for deployment to Libera.
install: locations $(BIN_DIR)/st.cmd $(BIN_DIR)/runioc

clean:
	rm -f locations


# Preprocess the startup script to ensure that only the appropriate databases
# are included.  This is a very ad-hoc conditional build process: I'm sure we
# can do better...
#    The sed commands here process all the #if FF lines and also strip out
# comments and blank lines.
ifeq ($(BUILD_FF_SUPPORT),yes)
    ST_SED_COMMAND = '/^\#if FF /s///; /^\#/d; /^ *$$/d'
else
    ST_SED_COMMAND = '/^\#if FF /d; /^\#/d; /^ *$$/d'
endif
$(BIN_DIR)/st.cmd: st.cmd
	sed $(ST_SED_COMMAND) $< >$@
	@echo Created st.cmd

# The list of possible installation locations is derived from CONFIG.
LOC_SED_COMMAND = '/^([[:alpha:]][[:alnum:]])_IOC_.*$$/{s//\1/;p;}' 
locations: CONFIG
	sed -rn $(LOC_SED_COMMAND) $< | sort | uniq >$@

$(BIN_DIR)/runioc: runioc
	cp $< $@

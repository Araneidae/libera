# IOC startup script for Libera EPICS driver.
#
# This file will be preprocessed as part of the installation process: all 
# comment lines are removed, and conditional lines are processed. 
#

# Standard IOC registration process
dbLoadDatabase("dbd/ioc.dbd",0,0)
ioc_registerRecordDeviceDriver(pdbbase)

# Standard libera records used by all versions of this IOC
dbLoadRecords("db/libera.db", "${LIBERA_MACROS}")

# Fast feedback support: Diamond specific, only included if implemented.
#if FF dbLoadRecords("db/fastFeedback.db", "${LIBERA_MACROS}")

epicsEnvSet "IOCSH_PS1" "${DEVICE}> "
iocInit()

# IOC startup script for Libera EPICS driver.
dbLoadDatabase("dbd/ioc.dbd",0,0)
ioc_registerRecordDeviceDriver(pdbbase)
dbLoadRecords("db/libera.db", "${LIBERA_MACROS}")
iocInit()

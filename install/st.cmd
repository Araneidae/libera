# IOC startup script for Libera EPICS driver.

# Register all support components with EPICS
dbLoadDatabase("dbd/ioc.dbd",0,0)
ioc_registerRecordDeviceDriver(pdbbase)

# Load the records
dbLoadRecords("db/sensors.db", "DEVICE=${DEVICE}")
dbLoadRecords("db/libera.db",  "DEVICE=${DEVICE}")

dbLoadRecords("config/${DEVICE}.db", "DEVICE=${DEVICE}")

# Run EPICS
iocInit()


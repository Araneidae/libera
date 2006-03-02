TOP=../..

include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CXXFLAGS += -S
USR_CXXFLAGS += -Werror

# Make sure that we pick up the version from install/CONFIG.
USR_CXXFLAGS += -DLIBERA_VERSION='"$(shell \
    sed -n '/^VERSION=/{s///;p}' $(TOP)/install/CONFIG)"'

# The lines below defeat the -ansi flag.
#CMPLR=LAX
#CXXCMPLR=LAX
CONFORM_CFLAGS_STRICT = 
CONFORM_CXXFLAGS_STRICT = 


#=============================
# build an ioc application

PROD_IOC = ioc

# <name>.dbd will be created from <name>Include.dbd
DBD += ioc.dbd
DBD += device.dbd
DBD += libera.dbd


# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
ioc_SRCS += ioc_registerRecordDeviceDriver.cpp
ioc_SRCS += iocMain.cpp
ioc_SRCS += readFile.cpp
ioc_SRCS += drivers.cpp
ioc_SRCS += publish.cpp
ioc_SRCS += trigger.cpp
ioc_SRCS += firstTurn.cpp
ioc_SRCS += booster.cpp
ioc_SRCS += turnByTurn.cpp
ioc_SRCS += freeRun.cpp
ioc_SRCS += slowAcquisition.cpp
ioc_SRCS += postmortem.cpp
ioc_SRCS += waveform.cpp
ioc_SRCS += cordic.cpp
ioc_SRCS += convert.cpp
#ioc_SRCS += hardware.cpp
ioc_SRCS += hardware_cspi.cpp
ioc_SRCS += events.cpp
ioc_SRCS += support.cpp

ioc_SRCS += thread.cpp
#ioc_SRCS += hardware_events.cpp


ioc_LIBS += $(EPICS_BASE_IOC_LIBS) 


# Point to the Libera device driver include, libera.h.
USR_INCLUDES += -I$(TOP)/driver -I$(TOP)/cspi
USR_LIBS += cspi
cspi_DIR = $(CSPI)


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# The support module uses a pre-built lookup table.
support.o: divide-lookup.h

divide-lookup.h: ../divide-lookup.py
	python ../divide-lookup.py >divide-lookup.h


# iocMain should have the current version information defined in CONFIG.
iocMain.o: $(TOP)/install/CONFIG

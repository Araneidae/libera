TOP=../..

include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CXXFLAGS += -S
#USR_CXXFLAGS += -E
USR_CXXFLAGS += -Werror -Wno-comment

# Make sure that we pick up the version from install/CONFIG.
USR_CXXFLAGS += -DLIBERA_VERSION='"$(shell \
    sed -n '/^VERSION=/{s///;p}' $(TOP)/install/CONFIG)"'
# Compute a sensible format of build date
USR_CXXFLAGS += -DBUILD_DATE_TIME='"$(shell \
    date -u '+%Y-%m-%d %H:%M:%S UTC')"'

# The lines below defeat the -ansi flag: we want to use some gcc specific
# features (typeof, in particular).
CONFORM_CFLAGS_STRICT = 
CONFORM_CXXFLAGS_STRICT = 


#=============================
# build an ioc application

PROD_IOC = ioc

# ioc.dbd will be created from iocInclude.dbd
DBD += ioc.dbd
DBD += libera.dbd


# ioc_registerRecordDeviceDriver.cpp will be created from ioc.dbd
ioc_SRCS += ioc_registerRecordDeviceDriver.cpp

ioc_SRCS += iocMain.cpp
ioc_SRCS += device.cpp
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
ioc_SRCS += hardware.cpp
ioc_SRCS += events.cpp
ioc_SRCS += support.cpp
ioc_SRCS += thread.cpp
ioc_SRCS += persistent.cpp
ioc_SRCS += interlock.cpp
ioc_SRCS += sensors.cpp

ioc_LIBS += cspi
ioc_LIBS += $(EPICS_BASE_IOC_LIBS) 

# Point to the Libera device driver includes, libera.h and cspi.h.
USR_INCLUDES += -I$(TOP)/driver -I$(CSPI)


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


# The support module uses a pre-built lookup table.
support.o: support-lookup.h

%.h: ../%.py
	python $< >$@


# Force iocMain to be rebuilt every time so that the build date and time is
# current. 
iocMain.o: EMPTY
EMPTY:

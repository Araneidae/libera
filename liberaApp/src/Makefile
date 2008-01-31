TOP=../..

include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#USR_CXXFLAGS += -S
#USR_CXXFLAGS += -E
#USR_CXXFLAGS += -g -O0

USR_CXXFLAGS += -Werror 
USR_CXXFLAGS += -Wextra -Wno-unused-parameter

# Make sure that we pick up the version from install/CONFIG.
USR_CXXFLAGS += -DLIBERA_VERSION='"$(shell \
    sed -n '/^VERSION=/{s///;p}' $(TOP)/install/CONFIG)"'
# Compute a sensible format of build date
USR_CXXFLAGS += -DBUILD_DATE_TIME='"$(shell date '+%Y-%m-%d %H:%M')"'

# The lines below defeat the -ansi flag: we want to use some gcc specific
# features (typeof, in particular).
CONFORM_CFLAGS_STRICT = 
CONFORM_CXXFLAGS_STRICT = 

# SHARED_LIBRARIES=NO   <--- appears to do nothing
STATIC_BUILD = YES


#=============================
# build an ioc application

PROD_IOC = ioc


DBD += ioc.dbd                  # ioc.dbd will be created from iocInclude.dbd
DBD += libera.dbd


# ioc_registerRecordDeviceDriver.cpp will be created from ioc.dbd
ioc_SRCS += ioc_registerRecordDeviceDriver.cpp

ioc_SRCS += device.cpp          # EPICS record interface
ioc_SRCS += publish.cpp         # Support for publishing to EPICS
ioc_SRCS += trigger.cpp         # I/O Intr support and EPICS interlocking
ioc_SRCS += firstTurn.cpp       # First Turn (FT) mode
ioc_SRCS += booster.cpp         # Decimated data (BN) mode
ioc_SRCS += turnByTurn.cpp      # Long turn-by-turn (TT) mode
ioc_SRCS += freeRun.cpp         # Free running turn-by-turn (FR) mode
ioc_SRCS += slowAcquisition.cpp # Slow acquisition (SA) mode
ioc_SRCS += postmortem.cpp      # Postmortem (PM) mode
ioc_SRCS += waveform.cpp        # Waveform management support
ioc_SRCS += cordic.cpp          # Fast computation of sqrt(x*x+y*y)
ioc_SRCS += convert.cpp         # Positions configuration and conversion
ioc_SRCS += configure.cpp       # Switches and attenuation configuration
ioc_SRCS += timestamps.cpp      # Synchronisation and timestamp support
ioc_SRCS += hardware.cpp        # Driver level hardware interface
ioc_SRCS += events.cpp          # Reception of trigger and other device events
ioc_SRCS += numeric.cpp         # Fast arithmetic support
ioc_SRCS += thread.cpp          # Simple support for pthreads
ioc_SRCS += persistent.cpp      # Persistent configuration settings
ioc_SRCS += interlock.cpp       # Machine protection interlock
ioc_SRCS += sensors.cpp         # Machine state sensors
ioc_SRCS += conditioning.cpp    # Signal conditioning control

ioc_SRCS += iocMain.cpp         # Ioc startup and configuration

ifeq ($(BUILD_FF_SUPPORT),yes)
USR_CXXFLAGS += -DBUILD_FF_SUPPORT
ioc_SRCS += fastFeedback.cpp    # Fast feedback register access
endif

ioc_LIBS += $(EPICS_BASE_IOC_LIBS) 

# Point to the device driver includes
USR_INCLUDES += -I$(LIBERA_PLL)


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


# The numeric module uses a pre-built lookup table.
numeric.o: numeric-lookup.h

# First turn also depends on a prebuilt table
firstTurn.o: filter-header.h

%.h: ../%.py
	$(PYTHON) $< >$@


# Force iocMain to be rebuilt every time so that the build date and time is
# current. 
iocMain.o: EMPTY
EMPTY:

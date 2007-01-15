TOP=..
include $(TOP)/configure/CONFIG

SHARED_LIBRARIES=NO
CONFORM_CFLAGS_STRICT = 
CONFORM_CXXFLAGS_STRICT = 

USR_CFLAGS = -DEBPP -DCORDIC_IGNORE_GAIN -I$(CSPI)/driver -I$(CSPI)/msp -std=gnu99

LIBRARY_IOC = cspi

cspi_SRCS = cordic.c cspi.c ebpp.c

include $(TOP)/configure/RULES

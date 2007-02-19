TOP=../..
include $(TOP)/configure/CONFIG

SHARED_LIBRARIES=NO
CONFORM_CFLAGS_STRICT = 
CONFORM_CXXFLAGS_STRICT = 

cspidir = ../../../cspi
drvdir = $(cspidir)/driver

USR_CFLAGS += -DEBPP -D_REENTRANT -I$(drvdir) -I$(cspidir)
USR_CFLAGS += -Werror -Wno-comment -I$(CSPI)/driver -std=gnu99

PROD_IOC = ldscd

ldscd_SRCS += dscd.c dscd_impl.c dsc_fpga.c cordic_dsc.c

ldscd_LIBS += cspi

include $(TOP)/configure/RULES

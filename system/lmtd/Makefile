TOP=../..
include $(TOP)/configure/CONFIG

SHARED_LIBRARIES=NO
CONFORM_CFLAGS_STRICT = 
CONFORM_CXXFLAGS_STRICT = 

DEBUG = 0

USR_CFLAGS += -Werror -Wno-comment -std=gnu99 -DDEBUG=$(DEBUG)

USR_INCLUDES += -I$(CSPI)/driver -I$(CSPI) 

PROD_IOC = lmtd lstd

lmtd_SRCS += lmtd.c
lstd_SRCS += lstd.c

include $(TOP)/configure/RULES

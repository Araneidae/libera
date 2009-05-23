TOP=..
include $(TOP)/configure/CONFIG

SHARED_LIBRARIES=NO
STATIC_BUILD = YES

CONFORM_CFLAGS_STRICT = 

USR_CFLAGS += -march=armv5te 
USR_CFLAGS += -std=gnu99
USR_CFLAGS += -Werror -Wall -Wextra -Wno-unused-parameter

USR_INCLUDES += -I$(TOP)/Include

PROD_IOC = healthd

healthd_SRCS += healthd.c

include $(TOP)/configure/RULES

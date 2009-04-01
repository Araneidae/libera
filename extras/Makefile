TOP=..
include $(TOP)/configure/CONFIG

SHARED_LIBRARIES=NO
STATIC_BUILD = YES

CONFORM_CFLAGS_STRICT = 
CONFORM_CXXFLAGS_STRICT = 

USR_CFLAGS += -march=armv5te 
USR_CFLAGS += -std=gnu99
USR_CFLAGS += -Werror -Wall -Wextra -Wno-unused-parameter

USR_INCLUDES += -I$(TOP)/Include

# PROD_IOC = probe-config test
# 
# probe-config_SRCS += probe-config.c
# test_SRCS += probe-config.c

PROD_IOC = monitor

monitor_SRCS += monitor.c

include $(TOP)/configure/RULES

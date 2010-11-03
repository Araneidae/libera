TOP=..
include $(TOP)/configure/CONFIG

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

TOP=..
include $(TOP)/configure/CONFIG

USR_CFLAGS += -march=armv5te
USR_CFLAGS += -std=gnu99
USR_CFLAGS += -Werror -Wall -Wextra -Wno-unused-parameter

USR_INCLUDES += -I$(TOP)/Include

PROD_IOC = healthd

healthd_SRCS += healthd.c

include $(TOP)/configure/RULES

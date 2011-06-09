TOP=..
include $(TOP)/configure/CONFIG

USR_CFLAGS += -march=armv5te
USR_CFLAGS += -std=gnu99
USR_CFLAGS += -Werror -Wall -Wextra -Wno-unused-parameter

PROD_IOC = fp

fp_SRCS += fp.c

include $(TOP)/configure/RULES

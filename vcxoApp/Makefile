TOP=..
include $(TOP)/configure/CONFIG

USR_CFLAGS += -march=armv5te
USR_CFLAGS += -std=gnu99
USR_CFLAGS += -Werror -Wall -Wextra -Wno-unused-parameter

PROD_IOC = vcxo

vcxo_SRCS += vcxo.c

include $(TOP)/configure/RULES

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

PROD_IOC = clockPll

clockPll_SRCS += clockPll.c     # Entry point for clockPll and communication
clockPll_SRCS += controller.c   # PLL controller framework
clockPll_SRCS += machineClock.c # Controller definitions for MC clock
clockPll_SRCS += systemClock.c  # Controller definitions for SC clock

include $(TOP)/configure/RULES

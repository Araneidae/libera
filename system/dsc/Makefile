cspidir = ../../cspi
drvdir = $(cspidir)/driver

DEBUG = 0

CROSS_COMPILE = arm-linux-
CC = gcc 
CFLAGS = -std=gnu99 -Wall -Werror

CPPFLAGS = -DEBPP -D_REENTRANT -I$(drvdir) -I$(cspidir)
LDFLAGS = -L../../lib/linux-arm
LIBS = -lpthread -lcspi -lm


PATH := $(PATH):/home/targetOS/linux-arm/host/x86-linux/gcc_3.3.3/bin

#
# -------- configuration section: end --------
#

vpath %.h $(drvdir) $(cspidir)

CC := $(CROSS_COMPILE)$(CC)
AR := $(CROSS_COMPILE)$(AR)

# Release version
RELEASE_VERSION := $(shell /bin/date +%Y%m%d)
CPPFLAGS += -DRELEASE_VERSION=$(RELEASE_VERSION)

# Debugging and optimization flags
ifndef DEBUG
	DEBUG = 0
endif
ifeq ($(DEBUG),0)
	CFLAGS += -O2
	CPPFLAGS += -DNDEBUG
else
	CFLAGS += -O -g		# "-O" is needed to expand inlines
	CPPFLAGS += -DDEBUG=$(DEBUG)
endif

SRCS = dscd.c dscd_impl.c dsc_fpga.c cordic_dsc.c
OBJS = $(SRCS:%.c=%.o)


.PHONY : all clean


all: ldscd


ldscd: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f *.o ldscd .depend


$(OBJS): .depend


dep .depend:
	@echo Making dep and .depend
	$(CC) -MM $(CPPFLAGS) *.c >.depend


ifeq (.depend, $(wildcard .depend))
include .depend
endif

# EOF

# $Id: Makefile,v 1.19 2006/01/12 08:06:49 miha Exp $
#
# Copyright (C) 2004, 2005 Instrumentation Technologies
# See the COPYING file for copying conditions.
#
#
# GNU Makefile to build the CSPI library test programs.
# If you have GNU Make installed, type 
#
#   make
#   make install
#   make clean
#
# on the command line to build the programs.
# To change the default parameters, edit the configuration
# section below.
#

# 
# -------- configuration section: start --------
#

#
# TARGET: Set to EBPP or BBFP.
#
TARGET = EBPP

#
# drvdir: Where to look for Libera GNU/Linux driver .h files.
#
drvdir = ../../driver

#
# cspidir: Where to look for CSPI .h files.
#
cspidir = ..

#
# DEBUG: Debug level. Set to non-zero for debug build or
# anything else (zero or commented) for non-debug build.
# Note: CPPFLAGS is appended the DEBUG and NDEBUG flag in
# debug and non-debug build, respectively.
#
DEBUG = 0

#
# CROSS_COMPILE: Cross compiler prefix to prepend to CC and CXX.
#
CROSS_COMPILE = arm-linux-

#
# CC: The command to invoke C compiler.
#
CC = gcc 

#
# CXX: The command to invoke C++ compiler.
#
CXX = g++

#
# CFLAGS: Options to pass to the C compiler.
#
CFLAGS = -std=gnu99 -Wall

#
# CXXFLAGS: Options to pass to the C++ compiler.
#
CXXFLAGS = -Wall

#
# CPPFLAGS: Options to pass to the preprocessor.
#
CPPFLAGS = -D$(TARGET) -I$(drvdir) -I$(cspidir)

#
# LDFLAGS: Options to pass to the linker.
#
LDFLAGS = -L$(cspidir)

#
# LIBS: Additional libs to pass to the linker.
#
LIBS = -lcspi -lpthread

#
# INSTALL: The pathname of the 'install' command for installing
# a file into the system.
#
INSTALL = /usr/bin/install

#
# prefix: A prefix used in constructing the installation
# pathnames below.
#
prefix = /opt

#
# bindir: The directory to install the application in.
#
bindir = $(prefix)/bin

#
# -------- configuration section: end --------
#

vpath %.h $(drvdir) $(cspidir)
vpath lib% $(cspidir)

CC := $(CROSS_COMPILE)$(CC)
CXX := $(CROSS_COMPILE)$(CXX)

# Release version
RELEASE_VERSION := $(shell /bin/date +%Y%m%d)
CPPFLAGS += -DRELEASE_VERSION=$(RELEASE_VERSION)

# Debugging and optimization flags
ifndef DEBUG
	DEBUG = 0
endif
ifeq ($(DEBUG),0)
	DEBUG_FLAGS += -O2
	CPPFLAGS += -DNDEBUG
else
	DEBUG_FLAGS += -O -g	# "-O" is needed to expand inlines
	CPPFLAGS += -DDEBUG=$(DEBUG)
endif

CFLAGS  += $(DEBUG_FLAGS)
CXXFLAGS += $(DEBUG_FLAGS)

ifeq ($(TARGET),EBPP)
	C_TARGETS := test-event acquire
	CXX_TARGETS := libera
endif
ifeq ($(TARGET),BBFP)
	C_TARGETS := test-bbfp
	#CXX_TARGETS :=
endif


C_SRCS := $(addsuffix .c,$(C_TARGETS))
CXX_SRCS := $(addsuffix .cpp,$(CXX_TARGETS))


.PHONY : all install dep


all: $(C_TARGETS) $(CXX_TARGETS)


$(C_TARGETS): %: %.o
	$(CC) $(LDFLAGS) -o $@ $< $(LIBS)


$(CXX_TARGETS): %: %.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)


$(C_TARGETS) $(CXX_TARGETS): -lcspi


$(C_SRCS) $(CXX_SRCS): .depend


install: install-$(TARGET)


install-EBPP:
	$(CROSS_COMPILE)strip acquire libera
	$(INSTALL) -d -o root -g root $(bindir)
	$(INSTALL) -m 755 -o root -g root libera $(bindir)
	$(INSTALL) -m 664 -o root -g root gain.conf env.conf $(bindir)


install-BBFP:
	@echo Empty target!


clean:
	-$(RM) *~ *.o .depend $(C_TARGETS) $(CXX_TARGETS)


dep .depend:
	$(CXX) -MM $(CPPFLAGS) *.c *.cpp >.depend


ifeq (.depend, $(wildcard .depend))
include .depend
endif

# EOF

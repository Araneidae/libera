TOP=../..

include $(TOP)/configure/CONFIG


USR_CXXFLAGS += -Werror -Wno-comment
USR_CXXFLAGS += -DEBPP

PROD_IOC = libera
libera_SRCS += libera.cpp
libera_LIBS += cspi

USR_INCLUDES += -I$(CSPI)/driver -I$(CSPI)


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

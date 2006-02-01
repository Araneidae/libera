#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) configure liberaApp docs
include $(TOP)/configure/RULES_TOP

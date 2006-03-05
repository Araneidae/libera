#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) configure cspi liberaApp 
include $(TOP)/configure/RULES_TOP

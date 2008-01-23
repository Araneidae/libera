#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure 
DIRS += system/lmtd liberaApp 
DIRS += install
include $(TOP)/configure/RULES_TOP

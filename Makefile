#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure 
DIRS += clockPllApp
DIRS += liberaApp 
DIRS += extras
DIRS += install
include $(TOP)/configure/RULES_TOP

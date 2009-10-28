#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += clockPllApp
DIRS += healthdApp
DIRS += liberaApp
DIRS += extras
DIRS += install_d
include $(TOP)/configure/RULES_TOP

#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += clockPllApp
DIRS += healthdApp
DIRS += vcxoApp
DIRS += liberaApp
DIRS += extras
DIRS += base
DIRS += install_d
DIRS += opi
include $(TOP)/configure/RULES_TOP

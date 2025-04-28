# Project Name
TARGET = WarpDrive

# Sources
CPP_SOURCES = WarpDrive.cpp oscillator.cpp encodercontrol.cpp eulerssynthesis.cpp adsr.cpp


# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

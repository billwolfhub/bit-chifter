# Bitchshifter for the Synthux Simple Touch
TARGET = Bitchshifter
CPP_SOURCES = simple_touch.cpp engine/engine.cpp
LIBDAISY_DIR = ../libDaisy
APP_TYPE = BOOT_SRAM
C_INCLUDES += -Iengine
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Makefile to build Teensy 3.x programs without the need to use Arduino/Teensyduino IDE
# It can opened as Makefile project in CLion.
#
# original source: https://github.com/apmorton/teensy-template/blob/master/Makefile
# see also: https://github.com/PaulStoffregen/cores/blob/master/teensy3/Makefile
#

# The name of your project (used to name the compiled .hex file)
TARGET = $(notdir $(CURDIR))

# The teensy version to use, 30, 31, 32, 35, 36, or LC
TEENSY = 32

# Set to 24000000, 48000000, or 96000000 to set CPU core speed
TEENSY_CORE_SPEED = 48000000

# Some libraries will require this to be defined
# If you define this, you will break the default main.cpp
# ARDUINO = 10600

# configurable options
OPTIONS = -DUSB_SERIAL -DLAYOUT_US_ENGLISH

# directory to build in
BUILDDIR = $(abspath $(CURDIR)/build)

#************************************************************************
# Location of Teensyduino utilities, Toolchain, and Arduino Libraries.
# To use this makefile without Arduino, copy the resources from these
# locations and edit the pathnames.  The rest of Arduino is not needed.
#************************************************************************

# path location for Teensy Loader, teensy_post_compile and teensy_reboot
TOOLSPATH = /Applications/Teensyduino.app/Contents/Java/hardware/tools

# path location for Teensy 3 core
# copied from https://github.com/PaulStoffregen/cores/tree/master/teensy3
# see scripts/update-teensy3-core.sh
COREPATH = teensy3

# path location for Arduino libraries
LIBRARYPATH = libraries

# path location for the arm-none-eabi compiler (normally it's shipped together with the Teensy tools)
# note: The shipped-with-Teensy gcc version is a bit outdated,
#       but it is compatible with the Teensy' core lib code (newer gcc might not be).
COMPILERPATH = $(TOOLSPATH)/arm/bin

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -Os -mthumb -ffunction-sections -fdata-sections -nostdlib -MMD $(OPTIONS) -DTEENSYDUINO=124 -DF_CPU=$(TEENSY_CORE_SPEED) -Isrc -I$(COREPATH)

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS =

# linker options
LDFLAGS = -Os -Wl,--gc-sections -mthumb

# additional libraries to link
LIBS = -lm

# compiler options specific to teensy version
ifeq ($(TEENSY), 30)
    CPPFLAGS += -D__MK20DX128__ -mcpu=cortex-m4
    LDSCRIPT = $(COREPATH)/mk20dx128.ld
    LDFLAGS += -mcpu=cortex-m4 -T$(LDSCRIPT)
# Teensy 3.1 and Teensy 3.2 are same (in terms of MCU type)
else ifeq ($(TEENSY), $(filter $(TEENSY),31 32))
    CPPFLAGS += -D__MK20DX256__ -mcpu=cortex-m4
    LDSCRIPT = $(COREPATH)/mk20dx256.ld
    LDFLAGS += -mcpu=cortex-m4 -T$(LDSCRIPT)
else ifeq ($(TEENSY), LC)
    CPPFLAGS += -D__MKL26Z64__ -mcpu=cortex-m0plus
    LDSCRIPT = $(COREPATH)/mkl26z64.ld
    LDFLAGS += -mcpu=cortex-m0plus -T$(LDSCRIPT)
    LIBS += -larm_cortexM0l_math
else ifeq ($(TEENSY), 35)
    CPPFLAGS += -D__MK64FX512__ -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
    LDSCRIPT = $(COREPATH)/mk64fx512.ld
    LDFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T$(LDSCRIPT)
    LIBS += -larm_cortexM4lf_math
else ifeq ($(TEENSY), 36)
    CPPFLAGS += -D__MK66FX1M0__ -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
    LDSCRIPT = $(COREPATH)/mk66fx1m0.ld
    LDFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T$(LDSCRIPT)
    LIBS += -larm_cortexM4lf_math
else
    # NOTE: The following must be indented with spaces (not with tabs) otherwise
    #       make (at least on macOS) fails with an error:
    #         Makefile:97: *** commands commence before first target.  Stop.
    $(error Invalid setting TEENSY=$(TEENSY))
endif

# set arduino define if given
ifdef ARDUINO
    CPPFLAGS += -DARDUINO=$(ARDUINO)
else
    CPPFLAGS += -DUSING_MAKEFILE
endif

# names for the compiler programs
CC = $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CXX = $(abspath $(COMPILERPATH))/arm-none-eabi-g++
OBJCOPY = $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
SIZE = $(abspath $(COMPILERPATH))/arm-none-eabi-size

# automatically create lists of the sources and objects
LC_FILES := $(wildcard $(LIBRARYPATH)/*/*.c)
LCPP_FILES := $(wildcard $(LIBRARYPATH)/*/*.cpp)
TC_FILES := $(wildcard $(COREPATH)/*.c)
TCPP_FILES := $(wildcard $(COREPATH)/*.cpp)
C_FILES := $(wildcard src/*.c)
CPP_FILES := $(wildcard src/*.cpp)
INO_FILES := $(wildcard src/*.ino)

# include paths for libraries
L_INC := $(foreach lib,$(filter %/, $(wildcard $(LIBRARYPATH)/*/)), -I$(lib))

SOURCES := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(INO_FILES:.ino=.o) $(TC_FILES:.c=.o) $(TCPP_FILES:.cpp=.o) $(LC_FILES:.c=.o) $(LCPP_FILES:.cpp=.o)
OBJS := $(foreach src,$(SOURCES), $(BUILDDIR)/$(src))

default: help

all: hex

build: check-config check-car generate-version $(TARGET).elf

hex: $(TARGET).hex

post_compile: $(TARGET).hex
	@$(abspath $(TOOLSPATH))/teensy_post_compile -file="$(basename $<)" -path=$(CURDIR) -tools="$(abspath $(TOOLSPATH))"

reboot:
	@-$(abspath $(TOOLSPATH))/teensy_reboot

upload: post_compile reboot

$(BUILDDIR)/%.o: %.c
	@echo -e "[CC]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.cpp
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.ino
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -x c++ -include Arduino.h -c "$<"

$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo -e "[LD]\t$@"
	@$(CC) $(LDFLAGS) -o "$@" $(OBJS) $(LIBS)

%.hex: %.elf
	@echo -e "[HEX]\t$@"
	@$(SIZE) "$<"
	@$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"

# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	@echo "Cleaning..."
	@rm -rf "$(BUILDDIR)"
	@rm -f "$(TARGET).elf" "$(TARGET).hex"

update-teensy3-core:
	@echo "Updating teensy3..."
	./scripts/update-teensy3-core.sh

download-teensy-tools:
	@echo "Downloading Teensy tools for your OS..."
	./scripts/download-teensy-tools.sh

help:
	@echo "Use 'build' to compile project and 'upload' to move it to Teensy!"
	@echo "Specify target platform via 'make target-car=CAR_NAME build'."
	@echo "You can also use 'init' to generate empty config file."

check-config:
	@if [ ! -f ./src/config.h ]; then \
		echo "Generating 'src/config.h' as it is missing ..."; \
		echo "// Config file for Teensy board\n\
// Fill in parameters of a target car. You can find them on the platform\n\
// and/or in redmine.\n\
// Steering\n\
#define pwm_str_center_value        // Straight\n\
#define pwm_str_lowerlimit          // Lowest possible number\n\
#define pwm_str_upperlimit          // Largest possible number\n\
\n\
// Values for switching auto<->manual modes\n\
#define pwm_str_center_lower        // set this to pwm_str_center_value - 300\n\
#define pwm_str_center_upper        // set this to pwm_str_center_value + 300\n\
\n\
// Throttle\n\
#define pwm_thr_center_value        // Calm state\n\
#define pwm_thr_lowerlimit          // Lowest possible number\n\
#define pwm_thr_upperlimit          // Largest possible number\n\
\n\
// Values for switching auto<->manual modes\n\
#define pwm_thr_center_lower        // set this to pwm_thr_center_value - 200\n\
#define pwm_thr_center_upper        // set this to pwm_thr_center_value + 200" > src/config.h; \
		echo "Config file generated. Fill in the parameters." >&2; \
		exit 1; \
	fi;

check-car:
ifdef target-car
	@echo "Building project for" $(target-car)
else
	@echo "Building project for UNDEFINED car" >&2
endif

generate-version:
	@echo "Generating 'version.h' ..."
ifdef target-car
	@./generate-version.sh "$(target-car)"
else
	@./generate-version.sh "undefined"
endif

# see https://www.gnu.org/software/make/manual/html_node/Phony-Targets.html
.PHONY: clean all build hex post_compile reboot
.PHONY: update-teensy3-core download-teensy-tools
.PHONY: help check-config check-car generate-version

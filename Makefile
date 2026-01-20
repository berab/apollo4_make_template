#******************************************************************************
#
# Makefile - Rules for building the libraries, examples and docs.
#
# Copyright (c) 2024, Ambiq Micro, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# Third party software included in this distribution is subject to the
# additional license terms as defined in the /docs/licenses directory.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# This is part of revision release_sdk_4_5_0-a1ef3b89f9 of the ../AmbiqSuite Development Package.
#
#******************************************************************************
-include board-defs.mk
# Include rules specific to this board

# All makefiles use this to find the top level directory.
SDK_PATH := $(AMBIQSUITE)
SWROOT ?= $(SDK_PATH)

# Include rules for building generic examples.
include $(SWROOT)/makedefs/example.mk

# JLINK SERVER:
# JLinkGDBServerCLExe -singlerun -noguio -port 61234 -device AMAP42KL-KBR

TARGET := hello_world
COMPILERNAME := gcc
PROJECT := hello_world_gcc
CONFIG := bin
GDB := gdb-multiarch
GDB_CONFIG := .gdbinit
GDB_PORT := 61234

SHELL:=/bin/bash

# Enable printing explicit commands with 'make VERBOSE=1'
ifneq ($(VERBOSE),1)
Q:=@
endif

#### Setup ####

TOOLCHAIN ?= arm-none-eabi
CPU = cortex-m4
FPU = fpv4-sp-d16
# Default to FPU hardware calling convention.  However, some customers and/or
# applications may need the software calling convention.
#FABI = softfp
FABI = hard

LINKER_FILE := ./gcc/linker_script.ld
STARTUP_FILE := ./gcc/startup_$(COMPILERNAME).c

#### Required Executables ####
CC = $(TOOLCHAIN)-gcc
GCC = $(TOOLCHAIN)-gcc
CPP = $(TOOLCHAIN)-cpp
LD = $(TOOLCHAIN)-ld
CP = $(TOOLCHAIN)-objcopy
OD = $(TOOLCHAIN)-objdump
RD = $(TOOLCHAIN)-readelf
AR = $(TOOLCHAIN)-ar
SIZE = $(TOOLCHAIN)-size
RM = $(shell which rm 2>/dev/null)

EXECUTABLES = CC LD CP OD AR RD SIZE GCC
K := $(foreach exec,$(EXECUTABLES),\
        $(if $(shell which $($(exec)) 2>/dev/null),,\
        $(info $(exec) not found on PATH ($($(exec))).)$(exec)))
$(if $(strip $(value K)),$(info Required Program(s) $(strip $(value K)) not found))

ifneq ($(strip $(value K)),)
all clean:
	$(info Tools $(TOOLCHAIN)-$(COMPILERNAME) not installed.)
	$(RM) -rf bin
else

DEFINES+= -DAM_PACKAGE_BGA
DEFINES+= -DAM_PART_$(CFAMILY)
DEFINES+= -Dgcc

INCLUDES = -I$(SDK_PATH)
INCLUDES+= -I$(SDK_PATH)/CMSIS/ARM/Include
INCLUDES+= -I$(SDK_PATH)/CMSIS/AmbiqMicro/Include
INCLUDES+= -I$(SDK_PATH)/devices
INCLUDES+= -I$(SDK_PATH)/mcu/$(FAMILY)
INCLUDES+= -I$(SDK_PATH)/mcu/$(FAMILY)/hal
INCLUDES+= -I$(SDK_PATH)/utils
INCLUDES+= -I$(SDK_PATH)/boards/$(BOARD)/bsp
INCLUDES+= -Isrc
INCLUDES+= -Igcc

VPATH = $(SDK_PATH)/utils
VPATH+=:src
VPATH+=:gcc

SRC = am_resources.c
SRC += am_util_delay.c
SRC += am_util_id.c
SRC += am_util_stdio.c
SRC += am_util_string.c
SRC += hello_world.c
SRC += startup_gcc.c

CSRC = $(filter %.c,$(SRC))
ASRC = $(filter %.s,$(SRC))

OBJS = $(CSRC:%.c=$(CONFIG)/%.o)
OBJS+= $(ASRC:%.s=$(CONFIG)/%.o)

DEPS = $(CSRC:%.c=$(CONFIG)/%.d)
DEPS+= $(ASRC:%.s=$(CONFIG)/%.d)

LIBS = $(SDK_PATH)/mcu/$(FAMILY)/hal/mcu/gcc/bin/libam_hal.a
LIBS += $(SDK_PATH)/boards/$(BOARD)/bsp/gcc/bin/libam_bsp.a

CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections -fomit-frame-pointer
CFLAGS+= -MMD -MP -std=c99 -Wall -g
CFLAGS+= -O0
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)
CFLAGS+= 

LFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
LFLAGS+= -nostartfiles -static
LFLAGS+= -Wl,--gc-sections,--entry,Reset_Handler,-Map,$(CONFIG)/$(TARGET).map
LFLAGS+= -Wl,--start-group -lm -lc -lgcc -lnosys $(LIBS) -Wl,--end-group
LFLAGS+= 

# Additional user specified CFLAGS
CFLAGS+=$(EXTRA_CFLAGS)

CPFLAGS = -Obinary

ODFLAGS = -S

#### Rules ####
all: directories $(CONFIG)/$(TARGET).bin $(GDB_CONFIG) run_jlink run_gdb

directories: $(CONFIG)

$(CONFIG):
	@mkdir -p $@

$(CONFIG)/%.o: %.c $(CONFIG)/%.d
	@echo "My config: $(CONFIG)"
	@echo " Compiling $(COMPILERNAME) $<"
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(CONFIG)/%.o: %.s $(CONFIG)/%.d
	@echo " Assembling $(COMPILERNAME) $<"
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(CONFIG)/$(TARGET).axf: $(OBJS) $(LIBS)
	@echo " Linking $(COMPILERNAME) $@"
	$(Q) $(CC) -Wl,-T,$(LINKER_FILE) -o $@ $(OBJS) $(LFLAGS)


$(CONFIG)/$(TARGET).bin: $(CONFIG)/$(TARGET).axf
	@echo " Copying $(COMPILERNAME) $@..."
	$(Q) $(CP) $(CPFLAGS) $< $@
	$(Q) $(OD) $(ODFLAGS) $< > $(CONFIG)/$(TARGET).lst
	$(Q) $(SIZE) $(OBJS) $(LIBS) $(CONFIG)/$(TARGET).axf >$(CONFIG)/$(TARGET).size

$(GDB_CONFIG):
	@echo " Creating .gdbinit..."
	@printf "file $(CONFIG)/$(TARGET).axf\ntarget remote localhost:$(GDB_PORT)\nload\nbreak main\ncontinue\nlay next\nlay next\nlay next\nlist\nnext" > $(GDB_CONFIG)

clean:
	@echo "Cleaning..."
	$(Q) $(RM) -rf $(CONFIG)
	$(Q) $(RM) -rf $(GDB_CONFIG)
$(CONFIG)/%.d: ;

$(SDK_PATH)/mcu/$(FAMILY)/hal/mcu/gcc/bin/libam_hal.a:
	$(MAKE) -C $(SDK_PATH)/mcu/$(FAMILY)/hal/mcu

$(SDK_PATH)/boards/$(BOARD)/bsp/gcc/bin/libam_bsp.a:
	$(MAKE) -C $(SDK_PATH)/boards/$(BOARD)/bsp

# Automatically include any generated dependencies
-include $(DEPS)
endif
.PHONY: all clean directories

run_jlink:
	JLinkGDBServerCLExe -singlerun -noguio -port $(GDB_PORT) -device $(DEVICE) >L /dev/null & 

run_gdb:
	$(GDB)


#4 Automatic makefile for GNUARM (C/C++)

#Target binary file name
TARGET	   = blinkf4

#Output format (elf,hex,bin)
FORMAT	= hex

#Optimalization [0,1,2,3,s]
# 0 - none optimalization, s - size optimalization 3 - most optimized
OPT 	?= 2

#MCU variant
#MCU_MAJOR_TYPE = f4
#MCU_MINOR_TYPE = 17VGT6
#MCU_MAJOR_TYPE = f1
#MCU_MINOR_TYPE = 07VBT6
MCU_MAJOR_TYPE = f2
MCU_MINOR_TYPE = 05VCT6

#USE SWD interface
PGM_SWD=n
PGM_SPEED_KHZ=3000


#For bootloader
USE_SBL_BOOTLOADER = n

#System stack size interrupts etc
MCU_SYSTEM_STACK_SIZE = 512

#Enable exceptions support
CPP_EXCEPTIONS = y

#Common flags 
COMMON_FLAGS = -Wall -Wextra -Wno-variadic-macros  -fshort-enums


#C compiler options
CFLAGS += $(COMMON_FLAGS)

#C++ compiler options 
CXXFLAGS += $(COMMON_FLAGS) -std=c++11

#Per file listing
LISTING = n

#Debug version
DEBUG ?=  y


#Source C files
SRC += $(wildcard *.c) 

#Source C++ files
CPPSRC += $(wildcard *.cpp)

#Source ASM files
ASRC += $(wildcard *.S)

ISIX_DIR=../../isixrtos/libisix
ISIXDRV_DIR=../../isixrtos/libisixdrvstm32
LIBFOUNDATION_DIR=../../isixrtos/libfoundation
LIBSTM32_DIR=../../isixrtos/libstm32
include $(ISIX_DIR)/isix.mk
include $(ISIXDRV_DIR)/isixdrv.mk
include $(LIBFOUNDATION_DIR)/libfoundation.mk
include $(LIBSTM32_DIR)/libstm32.mk
include $(LIBSTM32_DIR)/scripts/stm32.mk


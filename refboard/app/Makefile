#4 Automatic makefile for GNUARM (C/C++)

#Target binary file name
TARGET	   = brefboard

#Output format (elf,hex,bin)
FORMAT	= elf

#Optimalization [0,1,2,3,s]
OPT 	?= s

#MCU variant
MCU_MAJOR_TYPE = f4
MCU_MINOR_TYPE = 07ZET6

#Enable bootloader code
USE_SBL_BOOTLOADER = n
#System stack size interrupts etc
MCU_SYSTEM_STACK_SIZE = 512
#Enable exceptions support
CPP_EXCEPTIONS = n
#Per file listing
LISTING = n
#Debug version
DEBUG ?=  y
#Programer configuration
PGM_SWD = y
PGM_TYPE=stlink-v2-1


#USB driver type
ISIX_USBDRV_TYPE = stm32f2_device

#Common flags 
COMMON_FLAGS = -Wall -Wextra  -Wno-variadic-macros
#Configuration flags
COMMON_FLAGS += -DFUNCTION_MAIN_RETURN -D_HAVE_CONFIG_H
COMMON_FLAGS += -I../board/include

#C compiler options
CFLAGS += $(COMMON_FLAGS)
#C++ compiler options 
CXXFLAGS += $(COMMON_FLAGS)


CPPSRC += $(wildcard ../board/*.cpp)
CPPSRC += $(wildcard *.cpp)
SRC += $(wildcard *.c) $(wildcard tcpecho/*.c)
ASRC += $(wildcard *.S)


ISIX_BASE_DIR=../../isixrtos
ISIX_DIR=$(ISIX_BASE_DIR)/libisix
ISIXDRV_DIR=$(ISIX_BASE_DIR)/libisixdrvstm32
LIBFOUNDATION_DIR=$(ISIX_BASE_DIR)/libfoundation
LIBGFX_DIR=$(ISIX_BASE_DIR)/libgfx
LIBSTM32_DIR=$(ISIX_BASE_DIR)/libstm32
LIBTCPIP_DIR=$(ISIX_BASE_DIR)/libtcpip
LIBFSFAT_DIR=$(ISIX_BASE_DIR)/libfsfat
LIBUSB_DIR=$(ISIX_BASE_DIR)/libusb

include $(LIBTCPIP_DIR)/libtcpip.mk
include $(LIBSTM32_DIR)/libstm32.mk
include $(ISIXDRV_DIR)/isixdrv.mk
include $(LIBGFX_DIR)/libgfx.mk
include $(LIBFOUNDATION_DIR)/libfoundation.mk
include $(LIBUSB_DIR)/libusb.mk
include $(ISIX_DIR)/isix.mk
include $(LIBFSFAT_DIR)/libfsfat.mk
include $(LIBSTM32_DIR)/scripts/stm32.mk

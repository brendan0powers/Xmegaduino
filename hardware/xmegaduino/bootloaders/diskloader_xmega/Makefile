###############################################################################
# Makefile for DiskLoader
###############################################################################

## General Flags
PROJECT = DiskLoader
TARGET = DiskLoader.elf
CC = avr-gcc

# BOARD2
MCU = atxmega32a4
AVR_FREQ   = 32000000L

# Specify the Arduino model using the assigned PID.  This is used by Descriptors.c
#   to set PID and product descriptor string
# Arduino Leonardo PID
ARDUINO_MODEL_PID = 0x0034
# Arduino Micro PID
#ARDUINO_MODEL_PID = 0x0035

# Change if your programmer is different
AVRDUDE_PROGRAMMER = avrispmkII
AVRDUDE_PORT = usb	   

# program name should not be changed...
PROGRAM    = DiskLoader

AVRDUDE = avrdude
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) -p $(MCU)

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

override CFLAGS = -g -Wall -Os -mmcu=$(MCU) -DF_CPU=$(AVR_FREQ) -DARDUINO_MODEL_PID=$(ARDUINO_MODEL_PID) $(DEFS) -ffunction-sections -gdwarf-2 -fdata-sections -fno-split-wide-types

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

# Processor definitions
ifeq ($(MCU), atxmega16a4)
  BOOT_SECTION_START            =0x004000
endif
ifeq ($(MCU), atxmega32a4)
  BOOT_SECTION_START            =0x008000
endif
ifeq ($(MCU), atxmega64a1)
  BOOT_SECTION_START            =0x010000
endif
ifeq ($(MCU), atxmega64a3)
  BOOT_SECTION_START            =0x010000
endif
ifeq ($(MCU), atxmega64a4)
  BOOT_SECTION_START            =0x010000
endif
ifeq ($(MCU), atxmega128a1)
  BOOT_SECTION_START            =0x020000
endif
ifeq ($(MCU), atxmega128a3)
  BOOT_SECTION_START            =0x020000
endif
ifeq ($(MCU), atxmega128a4)
  BOOT_SECTION_START            =0x020000
endif
ifeq ($(MCU), atxmega192a1)
  BOOT_SECTION_START            =0x030000
endif
ifeq ($(MCU), atxmega192a3)
  BOOT_SECTION_START            =0x030000
endif
ifeq ($(MCU), atxmega256a1)
  BOOT_SECTION_START            =0x040000
endif
ifeq ($(MCU), atxmega256a3)
  BOOT_SECTION_START            =0x040000
endif
ifeq ($(MCU), atxmega256a3b)
  BOOT_SECTION_START            =0x040000
endif

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-gc-sections,-Map=DiskLoader.map,--section-start=.text=$(BOOT_SECTION_START),--relax
LDFLAGS += -Wl,--section-start=.boot-entry=$(shell python -c "print hex($(BOOT_SECTION_START)+4096-8)")
LDFLAGS += -nodefaultlibs -nostartfiles


## Intel Hex file production flags
HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Objects explicitly added by the user
LINKONLYOBJECTS = 

MODULES   := .
SRC_DIR   := $(addprefix src/,$(MODULES))
BUILD_DIR := $(addprefix build/,$(MODULES))

SRC       := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp))
OBJ       := $(patsubst src/%.cpp,build/%.o,$(SRC),%.S)
DEP		  := $(OBJ:%.o=%.d)
INCLUDES  := $(addprefix -I,$(SRC_DIR))

vpath %.cpp $(SRC_DIR)

.PHONY: all checkdirs clean

all: checkdirs $(TARGET) DiskLoader.hex DiskLoader.lss size

-include $(DEP)

checkdirs: $(BUILD_DIR)

$(BUILD_DIR):
	@mkdir -p $@

clean:
	@rm -rf build/
	@rm -f *.hex
	@rm -f *.elf
	@rm -f *.lss
	@rm -f *.map

define make-goal
$1/%.o: %.cpp
	$(CC) $(INCLUDES) $(CFLAGS) -c $$< -MD -o $$@
endef

$(foreach bdir,$(BUILD_DIR),$(eval $(call make-goal,$(bdir))))

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) $^ -o $@

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@
	
%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: $(TARGET)
	@echo
#	@avr-size -C --mcu=${MCU} ${TARGET}.elf

program: $(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) -B 5 -u -U flash:w:$(TARGET).hex

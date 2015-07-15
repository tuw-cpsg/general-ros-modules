#
# Makefile for AVR
#
# @date 06.07.2013
# @author Denise Ratasich
# 

#-------------------------------------------------------------------------
# project specific things
# change these definitions for new projects
#-------------------------------------------------------------------------

# fill in a project name (name of the hexfile generated)
PROJNAME = main

#-------------------------------------------------------------------------
# constants which you won't have to modify for each project
#-------------------------------------------------------------------------

# specify source files
SRC := $(wildcard src/*.c)
OBJS := $(SRC:.c=.o)

# target architecture
MCU = atmega328p

BINFORMAT = ihex

PORT = /dev/spidev0.0
MC = m328p
PROG = linuxspi

# Flags
CFLAGS  = -mmcu=$(MCU) -Wall -Iinclude -Os #-O0 -Wa,-a=$*.list
LDFLAGS	= -mmcu=$(MCU) #-Wl,-Map=$*.map 
OCFLAGS	= -O $(BINFORMAT)
#EEPFLAGS= --set-section-flags=.eeprom="alloc,load" --change-section-address .eeprom-0x810000

#-------------------------------------------------------------------------
# the targets
#-------------------------------------------------------------------------

all: $(PROJNAME).hex

#avr-objcopy -O ihex demo.elf demo.hex
%.hex: %.elf
	avr-objcopy $(OCFLAGS) $< $@

%.elf: $(OBJS)
	avr-gcc $(OBJS) $(LIBS) $(LDFLAGS) -o $@

%.o: 	%.c
	avr-gcc $(CFLAGS) -c -o $@ $<

# avrdude -c stk500v2 -p ATmega16 -P /dev/ttyUSB0 -e -U flash:w:demo.hex
flash: $(PROJNAME).hex
	sudo avrdude -c $(PROG) -p $(MC) -P $(PORT) -e -U flash:w:$(PROJNAME).hex

.PHONY: clean
clean:
	rm -f *.hex *.elf *.map *.list *.o *~

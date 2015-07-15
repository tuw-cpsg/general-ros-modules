ROS package uc
==============

This ROS node receives the ADC samples from the microcontroller
(ATmega328P on the Gertboard) over UART.


Description
-----------

ADC input 0 (PC0) is sampled by the microcontroller ATmega328P mounted
at the top of the robot. The microcontroller sends the samples to the
Raspberry Pi over the UART. The ROS node running on the Raspberry Pi
publishes all ADC values to ROS.

The microcontroller can be programmed through the Raspberry Pi (ISP),
thanks to Kevin Cuzner who implemented the SPI interface for AVRDUDE,
see https://github.com/kcuzner/avrdude. A Makefile is provided to
flash the µC with the firmware. The code is located in the folder
'firmware'.

Currently, only ADC0 is converted and published. A current sensor is
connected to this input (project EMC2, WP7.3 use case).


Wiring
------

Activate Gertboard / power supply for µC:
- jumper Pin 1 3V3 (J7) <-> Pin 2 3V3 (J7)

Sampling the current sensor (current sensor <-> Gertboard/µC):
- OUT (white) <-> PC0 (J28)
- VCC (red) <-> 5V (J24)
- GND (black) <-> GND (J28)

Programming the µC (Pi <-> µC):
- wired jumper GP8 <-> RESET Pin 5 ISP (J23)
- wired jumper GP9 <-> MISO Pin 1 ISP (J23)
- wired jumper GP10 <-> MOSI Pin 4 ISP (J23)
- wired jumper GP11 <-> SCK Pin 3 ISP (J23)

Reading samples from µC (Pi <-> µC):
- jumper GP14 <-> MCRX
- jumper GP15 <-> MXTX

Firmware for ATmega328P (on Gertboard)
======================================

This folder is not (or part of) a ROS package. It consists of the
firmware for the microcontroller ATmega328P on the Gertboard.

The microcontroller samples additional sensors connected to the AD
converter. The samples can be read from the UART.


UART Protocol
-------------

The 10bit ADC values (2 bytes) can be read via UART as follows:

1. start byte: 'S'
2. number of data bytes (number of connected sensors * 2)
3. ADC value (high byte before low byte)
4. end byte: 'E'
5. newline bytes: "\n\r"


UART Settings
-------------

115200 baud, 8N1

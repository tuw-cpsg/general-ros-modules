ROS drivers
===========

ROS nodes for additional hardware connected to the RaspberryPi, e.g.,
accelerometer. The nodes kxtf9, imu3000 and uc must run on the
RaspberryPi.


Available
---------

* kxtf9 :: KXTF9 (accelerometer)
* imu3000 :: IMU-3000 (gyroscope)
* uc :: ATmega328P (microcontroller on Gertboard)

See the docs of the drivers for details.


Dependencies
------------

* WiringPi library, see http://wiringpi.com/

If not present, the packages kxtf9, imu3000 and uc won't be built.


Communication with ROS
----------------------

The accelerometer KXTF9 and gyroscope IMU3000 are connected via I2C to
the RaspberryPi. The drivers only sample and publish the sensor values
periodically (see the documentation in the source files for parameter
settings). 

The RaspberryPi may be connected to another CPU running ROS. See the
following link(s) for help:

* http://wiki.ros.org/ROS/NetworkSetup


Wiring
------

Activate Gertboard / power supply for µC:
- jumper Pin 1 3V3 (J7) <-> Pin 2 3V3 (J7)

Sampling the current sensor (current sensor <-> Gertboard/µC):
- OUT (white) <-> PC0 (J28)
- VCC (red) <-> 5V (J24)
- GND (black) <-> GND (J28)

Sampling IMU3000 (IMU3000 <-> Gertboard/Pi):
- SDA Pin 22 (JP4) <-> GP0
- SCL Pin 20 (JP4) <-> GP1
- VCC Pin 23 (JP4) <-> 3.3V (J7)
- GND Pin 15 (JP4) <-> GND (J6)

Programming the µC (Pi <-> µC):
- wired jumper GP8 <-> RESET Pin 5 ISP (J23)
- wired jumper GP9 <-> MISO Pin 1 ISP (J23)
- wired jumper GP10 <-> MOSI Pin 4 ISP (J23)
- wired jumper GP11 <-> SCK Pin 3 ISP (J23)

Reading samples from µC (Pi <-> µC):
- jumper GP14 <-> MCRX
- jumper GP15 <-> MXTX

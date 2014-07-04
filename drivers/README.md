ROS drivers
===========

ROS nodes for additional hardware connected to the RaspberryPi,
e.g. accelerometer.


Available
---------

* KXTF9 (accelerometer)
* IMU-3000 (gyro)


Dependencies
------------

* WiringPi library, see http://wiringpi.com/


Communication with ROS
----------------------

The accelerometer KXTF9 and gyroscope IMU3000 are connected via I2C to
the RaspberryPi. The drivers only sample and publish the sensor values
periodically (see the documentation in the source files for parameter
settings). 

The RaspberryPi may be connected to another CPU running ROS. See the
following link(s) for help:

* http://wiki.ros.org/ROS/NetworkSetup
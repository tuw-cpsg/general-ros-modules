/**
 * @file
 * @author Denise Ratasich
 * @date 06.07.2015
 *
 * @brief Samples battery sensors and publishes readings.
 *
 * subscribe: none
 * publish:
 * - am_battery [std_msgs/Float32] (current consumed by the battery in
 *     Ampere, positive when charging, negative when load is attached)
 **/

#include "ros/ros.h"
#include "additional_msgs/Float32Stamped.h"

#include <wiringPi.h>
#include <wiringSerial.h>

#include <stdexcept>

#define START_C	'S'
#define END_C	'E'

typedef enum {
  START,
  LEN,
  DATA_H,
  DATA_L,
  END,
  ERROR
} uart_state_t;

/** Sensor values which should be received. */
enum sensor {
  AM_BATTERY,
  NUM_SENSORS
};

/** File descriptor of UART interface. */
static int fd = -1;

/** Entry point. */
int main(int argc, char **argv)
{
  uart_state_t state = START;
  uint8_t len;
  uint8_t sensor;
  uint16_t adcs[NUM_SENSORS];
  uint32_t count = 0;

  try {
    // set up ROS
    ros::init(argc, argv, "uc");
    ros::NodeHandle n("~");

    ros::Publisher am_battery_pub = n.advertise<uc::Float32Stamped>("am_battery", 1);
    uc::Float32Stamped am_battery_msg;

    ros::Rate loop_rate(1);

    ROS_INFO("Initialization done.");

    // init UART
    fd = serialOpen("/dev/ttyAMA0", 115200);
    if (fd == -1)
      throw std::runtime_error("UART setup failed (/dev/ttyAMA0, 115200).");

    // loop
    while (ros::ok())
    {
      // get next char
      int c = serialGetchar(fd);
      if (c == -1)
	ROS_WARN("No character received for 10s.");

      // implement uart protocol: S-len-data-E, data: 2 bytes (hi-lo)
      switch(state) {
      case START:
	// wait on start character
	if (c == START_C)
	  state = LEN;
	break;
      case LEN:
	// read number of data bytes
	len = c;
	if (len != NUM_SENSORS*2) {
	  ROS_WARN("Length of data not expected. Abort message receive.");
	  state = ERROR;
	} else {
	  sensor = 0;
	  state = DATA_H;
	}
	break;
      case DATA_H:
	// save high byte of an adc value
	adcs[sensor] = ((uint8_t)c) << 8;
	state = DATA_L;
	break;
      case DATA_L:
	// save low byte
	adcs[sensor] |= (uint8_t)c;
	// check if there is another sensor value
	sensor++;
	if (sensor < len/2) {
	  state = DATA_H;
	} else {
	  state = END;
	}
	break;
      case END:
	if (c != END_C)
	  ROS_WARN("No end of message detected.");

	// do something with message -> publish to ROS
	am_battery_msg.header.seq = count;
	am_battery_msg.header.stamp = ros::Time::now();
	// Vin = ADC * 3.3 / 1024;
	// 0A ... Vin - Vcc/2; 122mV / 1A
	am_battery_msg.data = (((float)adcs[AM_BATTERY]) * 3.3 / 1024 - 3.3/2) / 0.122;

	am_battery_pub.publish(am_battery_msg);
	count++;

	ROS_DEBUG_STREAM("am_battery: " << am_battery_msg.data);

	loop_rate.sleep();

	serialFlush(fd); // delete old messages in buffer
	state = START; // wait on next message
	break;
      case ERROR:
	ROS_WARN("Reached error state.");
	// wait until next start character
	state = START;
	break;
      default:
	assert("Invalid state.");
	break;
      } // end switch(uart state)
    } // end while(ros::ok)
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Unhandled exception reached the top of main: "
		     << e.what() << " Application will now exit." << std::endl);
    serialClose(fd);
    return 1;
  }

  serialClose(fd);
  return 0;
} // end main()

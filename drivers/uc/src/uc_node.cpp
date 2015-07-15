/**
 * @file
 * @author Denise Ratasich
 * @date 06.07.2015
 *
 * @brief Receives ADC values from uC and publishes readings.
 *
 * subscribe: none
 *
 * publish:
 * - adc_voltage [additional_msgs/Float32MultiArrayStamped] (voltage
 *   measured at the ADC inputs of the controller)
 *
 * parameters:
 * - ~pub_rate The publishing rate of ADC values. Default value is
 *   10Hz.
 **/

#include "ros/ros.h"
#include "additional_msgs/Float32MultiArrayStamped.h"

#include <wiringPi.h>
#include <wiringSerial.h>

#include <stdexcept>

#define START_C	'S'
#define END_C	'E'

/** ATmega328P-28PDIP has 6 ADC inputs. */
#define ADC_VOLTAGE_NUM	6

typedef enum {
  START,
  LEN,
  DATA_H,
  DATA_L,
  END,
  ERROR
} uart_state_t;

/** File descriptor of UART interface. */
static int fd = -1;

/**< Publishing rate, default: 10Hz. */
static int pub_rate = 10;

/**
 * @brief Checks whether the user has set parameters over the
 * parameter server, changes the defaults if any.
 */
void applyParameters (void)
{
  // check for parameters (e.g. I2C settings)
  ros::NodeHandle n_("~");

  // ROS params ---
  if (n_.hasParam("pub_rate"))
  {
    n_.getParam("pub_rate", pub_rate);
    ROS_INFO("Publishing rate set to %d Hz.", pub_rate);
  }
  else
  {
    ROS_INFO("Publishing rate set to default %d Hz.", pub_rate);
  }
}

/** Entry point. */
int main(int argc, char **argv)
{
  uart_state_t state = START;
  uint8_t len;
  uint8_t sensor;
  uint16_t adcs[ADC_VOLTAGE_NUM];
  uint32_t count = 0;

  try {
    // set up ROS
    ros::init(argc, argv, "uc");
    if (ros::this_node::getNamespace() == "/")
      ROS_WARN("Started in the global namespace.");

    applyParameters();

    ros::NodeHandle n("~");

    // ROS communication
    ros::Publisher adc_voltage_pub = n.advertise<additional_msgs::Float32MultiArrayStamped>("adc_voltage", 1);
    additional_msgs::Float32MultiArrayStamped adc_voltage_msg;
    adc_voltage_msg.layout.data_offset = 0;
    ros::Rate loop_rate(pub_rate);

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
	if (len >= ADC_VOLTAGE_NUM*2) {
	  ROS_WARN("Unexpected length of data. Abort message receive.");
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
	adc_voltage_msg.header.stamp = ros::Time::now();
	adc_voltage_msg.data.clear();
	for (int i = 0; i < len/2; i++) {
	  // Vin = ADC * 3.3 / 1024;
	  float vin = ((float)adcs[i]) * 3.3 / 1024;
	  adc_voltage_msg.data.push_back(vin);
	}
	adc_voltage_pub.publish(adc_voltage_msg);

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

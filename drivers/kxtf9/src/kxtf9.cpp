/**
 * @file kxtf9.cpp
 * @date 30.10.2013
 * @author Denise Ratasich
 * 
 * @brief ROS node reading and publishing the sensor values of KXTF9,
 * a 3-axis accelerometer.
 *
 * During initialization the bias is evaluated, so the robot should be
 * in a neutral position when starting this node. If this is not
 * possible the parameters ~offs_[x,y,z] should be set. During
 * operation this offset is then subtracted from the sensor's value.
 *
 * The frame_id is not yet set (you can set it with a parameter). On
 * the P3-AT the accelerometer is located near the base_link of the
 * rover, however the x-axis is aligned to the heading of the robot.
 *
 * - subscribe: none
 * - publish: acceleration [geometry_msgs/Vector3Stamped]
 *   (acceleration of all three axes in meters per square second)
 *
 * parameters
 * - ~pub_rate The publishing rate and sample rate of sensor
 *   data. Default value is 10Hz. This rate should be lower than the
 *   output data rate of the sensor, otherwise the same value will be
 *   published several times (the registers are read more often than
 *   they are updated!).
 * - ~frame_id Name of the coordinate system the accelerometer is
 *   placed in. Default value is "".
 * - ~output_data_rate The output data rate of the accelerometer, the
 *   update cycle of the acceleration data in the KXTF9
 *   registers. Should be higher than the pub_rate. Default value is
 *   200 Hz. Available options are: 25/50/100/200/400/800 Hz.
 * - ~offs_x The bias of the x-axis. Will be evaluated during
 *   calibration if not given.
 * - ~offs_y The bias of the y-axis. Will be evaluated during
 *   calibration if not given.
 * - ~offs_z The bias of the z-axis. Will be evaluated during
 *   calibration if not given.
 */

#include <stdlib.h>	// system, NULL
#include <stdint.h>	// int16_t, ...
#include <errno.h>	// errno
#include <string>
#include <stdexcept>
using namespace std;

// include low level library for RaspberryPi: wiringpi.com
#include <wiringPi.h>	// delay for calibration
#include <wiringPiI2C.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>	// message type for sensor values
		
// include the register definition of the accelerometer		       
#include "regKXTF9.h"

#define GRAVITY		(9.81)
/** Calculates the g-force given an ADC-value of the accelerometer. It
 *  is assumed that the resolution is 12bit -> 4096. */
#define COUNTS_TO_G(c,range)	((double)(c) * (range) * 2 / 4096)
#define COUNTS_TO_MPS2(c,range)	((double)(c) * (range) * 2 * GRAVITY / 4096)

/** File descriptor of I2C interface. */
static int fd = -1;

// variables for parameters
static uint8_t address = 0x0F;
static uint16_t range = 2;		// full range, default: +/- 2g
static int pub_rate = 10;		// publishing rate of data, default: 10Hz
static string frame_id = "";		// coordinate system of this accelerometer

static double offs_x = 0, offs_y = 0, offs_z = 0;

/**
 * @brief Calibrates the accelerometer, i.e. computes the offset.
 * 
 * The mean of 50 samples is taken to calculate the average of the
 * neutral position, which should be x = 0, y = 0, z = 1g (earth's
 * gravitation).
 */
void calibrate (void)
{
  int16_t acc_x, acc_y, acc_z;
  double sum_x = 0, sum_y = 0, sum_z = 0;
  const int NUM = 50;

  ROS_INFO("Calibrating ...");

  // in neutral position the values should be: (0,0,1)
  // the gravitation results in 1g
  for (int i = 0; i < NUM; i++)
  {
    // get new measurement data
    acc_x = wiringPiI2CReadReg8 (fd, XOUT_H) << 8;
    acc_x = acc_x | wiringPiI2CReadReg8 (fd, XOUT_L);
    acc_y = wiringPiI2CReadReg8 (fd, YOUT_H) << 8;
    acc_y = acc_y | wiringPiI2CReadReg8 (fd, YOUT_L);
    acc_z = wiringPiI2CReadReg8 (fd, ZOUT_H) << 8;
    acc_z = acc_z | wiringPiI2CReadReg8 (fd, ZOUT_L);

    // upper 12 bits hold the values, so shift them
    acc_x = acc_x >> 4;
    acc_y = acc_y >> 4;
    acc_z = acc_z >> 4;

    sum_x += acc_x;
    sum_y += acc_y;
    sum_z += acc_z;

    delay(5);
  }

  offs_x = COUNTS_TO_MPS2(sum_x / NUM, range);
  offs_y = COUNTS_TO_MPS2(sum_y / NUM, range);
  offs_z = COUNTS_TO_MPS2(sum_z / NUM, range) - GRAVITY;

  ROS_INFO_STREAM("Calibration done (offsets: " << offs_x << ", " << offs_y << ", " << offs_z << ").");
}

/**
 * @brief Configuration of KXTF9, changes the defaults to given
 * parameters if any.
 *
 * @note In this function control registers of the KXTF9 are set,
 * hence the PC1 bit in CTRL_REG1 is assumed to be '0'.
 */
void applyHardwareParameters (void)
{
  // check for parameters
  ros::NodeHandle n_("~");
  
  // update cycle of the acceleration data in the KXTF9 registers
  int odr = 200;	// default
  if (n_.hasParam("output_data_rate"))
  {
    n_.getParam("output_data_rate", odr);
    ROS_INFO("Output data rate set to %d Hz.", odr);
  }
  else
  {
    ROS_INFO("Output data rate set to default %d Hz.", odr);
  }

  // set ODR
  uint8_t dcr;
  switch (odr)
  {
  case 25: dcr = ODR_25; break;
  case 50: dcr = ODR_50; break;
  case 100: dcr = ODR_100; break;
  case 200: dcr = ODR_200; break;
  case 400: dcr = ODR_400; break;
  case 800: dcr = ODR_800; break;
  default: dcr = ODR_200; break;
  }
  
  // set output data rate
  if (wiringPiI2CWriteReg8 (fd, DATA_CTRL_REG, dcr) == -1)
    ROS_WARN_STREAM("Setting output data rate of KXTF9 failed. " << string(strerror(errno)));

  // set resolution to 12bit (fixed, no parameter provided)
  // set to operating mode (default: stand-by)
  if (wiringPiI2CWriteReg8 (fd, CTRL_REG1, (1<<PC1) | (1<<RES)) == -1)
    ROS_ERROR_STREAM("Configuration of KXTF9 failed. " << string(strerror(errno)));
}

/**
 * @brief Checks whether the user has set parameters over the
 * parameter server, changes the defaults if any.
 */
void applyOtherParameters (void)
{
  // check for parameters
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

  if (n_.hasParam("frame_id"))
  {
    n_.getParam("frame_id", frame_id);
    ROS_INFO_STREAM("Frame ID set to '" << frame_id << "'.");
  }

  // other configs --- 

  // bias for acc-values (should be used when this node isn't started
  // in neutral position (x,y,z)=(0,0,1g))
  if (n_.hasParam("offs_x") &&  
      n_.hasParam("offs_y") &&
      n_.hasParam("offs_z"))
  {
    ROS_INFO("Offsets taken from parameters.", pub_rate);
    n_.getParam("offs_x", offs_x);
    n_.getParam("offs_y", offs_y);
    n_.getParam("offs_z", offs_z);
  }
  else
  {
    // without parameters given for the bias, calibration is done
    // everytime the node is started
    calibrate();
  }
}

/**
 * @brief Initializes KXTF9 via I2C.
 */
void init (void)
{
  // loads the drivers into the kernel (creates i2c-1 file for
  // communication)
  if (system("gpio load i2c") != 0)
    throw runtime_error("Could not load I2C over gpio command.");

  // I2C address of KXTF9
  ros::NodeHandle n("~");
  if (n.hasParam("i2c_address"))
  {
    int paramAddr;
    n.getParam("i2c_address", paramAddr);
    address = (uint8_t) paramAddr;
    ROS_INFO("I2C address of KXTF9 set to 0x%02X.", address);
  }
  else
  {
    ROS_INFO("I2C address of KXTF9 set to default 0x%02X.", address);
  }

  // init I2C
  fd = wiringPiI2CSetup(address);
  if (fd == -1)
    throw runtime_error("I2C setup failed.");

  applyHardwareParameters();
  applyOtherParameters();
}

/**
 * @brief Entry point of this node.
 */
int main (int argc, char** argv)
{
  try
  {
    // inits for ROS
    ros::init(argc, argv, "kxtf9");
    if (ros::this_node::getNamespace() == "/")
      ROS_WARN("Started in the global namespace.");

    // init KXTF9
    try
    {
      init();
    }
    catch (exception& e)
    {
      ROS_ERROR_STREAM("Initialization via I2C failed. "
		       << e.what());
      return 1;
    }

    ROS_INFO("KXTF9 connected.");
    
    // ROS communication
    ros::NodeHandle n("~");
    ros::Publisher publisher = n.advertise<geometry_msgs::Vector3Stamped>("acceleration", 1);
    geometry_msgs::Vector3Stamped msg;
    msg.header.frame_id = frame_id;

    ros::Rate loop_rate(pub_rate);

    int count = 0;
    while (ros::ok())
    {
      // get new measurement data
      int16_t acc_x, acc_y, acc_z;
      acc_x = wiringPiI2CReadReg8 (fd, XOUT_H) << 8;
      acc_x = acc_x | wiringPiI2CReadReg8 (fd, XOUT_L);
      acc_y = wiringPiI2CReadReg8 (fd, YOUT_H) << 8;
      acc_y = acc_y | wiringPiI2CReadReg8 (fd, YOUT_L);
      acc_z = wiringPiI2CReadReg8 (fd, ZOUT_H) << 8;
      acc_z = acc_z | wiringPiI2CReadReg8 (fd, ZOUT_L);

      // upper 12 bits hold the values, so shift them
      acc_x = acc_x >> 4;
      acc_y = acc_y >> 4;
      acc_z = acc_z >> 4;

      // convert to g
      double x, y, z;
      x = COUNTS_TO_MPS2(acc_x,range) - offs_x;
      y = COUNTS_TO_MPS2(acc_y,range) - offs_y;
      z = COUNTS_TO_MPS2(acc_z,range) - offs_z;

      // fill message and publish
      msg.header.seq = count;
      msg.header.stamp = ros::Time::now();
      msg.vector.x = x;
      msg.vector.y = y;
      msg.vector.z = z;

      publisher.publish(msg);
      count++;

      ROS_DEBUG_STREAM("acc  x , y , z : "
		       << acc_x << " , "
		       << acc_y << " , "
		       << acc_z);

      loop_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_ERROR_STREAM("Unhandled exception reached the top of main: "
		     << e.what() << " Application will now exit." << endl);
    return 1;
  }

  ROS_INFO("Exit.");
  return 0;
}

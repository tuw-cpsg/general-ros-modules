/**
 * @file imu3000.cpp
 * @date 28.10.2013
 * @author Denise Ratasich
 * 
 * @brief ROS node reading and publishing the sensor values of
 * IMU3000, a 3-axis gyroscope.
 *
 * During initialization the bias is evaluated, so the robot should be
 * in a neutral position when starting this node. If this is not
 * possible the parameters ~offs_[x,y,z] should be set. During
 * operation this offset is then subtracted from the sensor's value.
 *
 * The frame_id is not yet set (you can set it with a parameter). On
 * the P3-AT the gyroscope is located near the base_link of the rover.
 *
 * - subscribe: none
 * - publish: angular_velocity [geometry_msgs/Vector3Stamped] (angular
 *   velocity of all three axes in radians per second)
 *
 * parameters
 * - ~pub_rate The publishing rate and sample rate of sensor
 *   data. Default value is 10Hz.
 * - ~frame_id Name of the coordinate system the gyroscope is
 *   placed in. Default value is "".
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
#include <cmath>
#include <string>
#include <stdexcept>
using namespace std;

#ifndef M_PI
#define PI	3.1415926535897932
#else
#define PI	M_PI
#endif

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "regIMU3000.h"

#define COUNTS_TO_DPS(c,range)		((double)(c) * (range) * 2 / 65536)
#define DPS_TO_COUNTS(dps,range)	((dps) * 65536 / 2 / (range))
#define COUNTS_TO_RPS(c,range)		((double)(c) * (range) * 2 * PI / 180 / 65536)
#define RPS_TO_COUNTS(rps,range)	((rps) * 180 * 65536 / 2 / (range) / PI)

/** File descriptor of I2C interface. */
static int fd = -1;

// variables for parameters
static uint8_t address = 0x68;
static uint16_t range = 250;		// full range, default: +/- 250°/s
static int pub_rate = 10;		// publishing rate of gyro-data, default: 10Hz
static string frame_id = "";		// coordinate system of this gyroscope

static double offs_x = 0, offs_y = 0, offs_z = 0;

/**
 * @brief Calibrates the gyroscope, i.e. computes the offset.
 * 
 * The mean of 50 samples is taken to calculate the average of the
 * neutral position, which should be x = 0, y = 0, z = 0, i.e. no
 * rotation.
 */
void calibrate (void)
{
  int16_t gyro_x, gyro_y, gyro_z;
  double sum_x = 0, sum_y = 0, sum_z = 0;
  const int NUM = 50;

  ROS_INFO("Calibrating ...");

  for (int i = 0; i < NUM; i++)
  {
    // get new measurement data
    gyro_x = wiringPiI2CReadReg8 (fd, GYRO_XOUT_H) << 8;
    gyro_x = gyro_x | wiringPiI2CReadReg8 (fd, GYRO_XOUT_L);
    gyro_y = wiringPiI2CReadReg8 (fd, GYRO_YOUT_H) << 8;
    gyro_y = gyro_y | wiringPiI2CReadReg8 (fd, GYRO_YOUT_L);
    gyro_z = wiringPiI2CReadReg8 (fd, GYRO_ZOUT_H) << 8;
    gyro_z = gyro_z | wiringPiI2CReadReg8 (fd, GYRO_ZOUT_L);

    sum_x += gyro_x;
    sum_y += gyro_y;
    sum_z += gyro_z;

    delay(5);
  }

  offs_x = COUNTS_TO_RPS(sum_x / NUM, range);
  offs_y = COUNTS_TO_RPS(sum_y / NUM, range);
  offs_z = COUNTS_TO_RPS(sum_z / NUM, range);

  ROS_INFO_STREAM("Calibration done (offsets: " << offs_x << ", " << offs_y << ", " << offs_z << ").");
}

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

  if (n_.hasParam("frame_id"))
  {
    n_.getParam("frame_id", frame_id);
    ROS_INFO_STREAM("Frame ID set to '" << frame_id << "'.");
  }

  // further configs for IMU-3000 ---

  // bias for gyro-values (should be used when this node isn't started
  // in neutral position (x,y,z)=(0,0,0), e.g. when the robot is
  // already driving)
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

  // offsets behave not as offsets?! seems like it would be
  // subtracted a multiple of the offset value set.
  /*
  // set offsets
  offs_x = RPS_TO_COUNTS(offs_x, range);
  offs_y = RPS_TO_COUNTS(offs_y, range);
  offs_z = RPS_TO_COUNTS(offs_z, range);
  wiringPiI2CWriteReg8 (fd, X_OFFS_USRH, 0x03);
  wiringPiI2CWriteReg8 (fd, X_OFFS_USRL, 0x5A);
  wiringPiI2CWriteReg8 (fd, Y_OFFS_USRH, offs_y >> 8);
  wiringPiI2CWriteReg8 (fd, Y_OFFS_USRL, offs_y);
  wiringPiI2CWriteReg8 (fd, Z_OFFS_USRH, offs_z >> 8);
  wiringPiI2CWriteReg8 (fd, Z_OFFS_USRL, offs_z);
  */
}

/**
 * @brief Initializes IMU-3000 via I2C.
 */
void init (void)
{
  // loads the drivers into the kernel
  if (system("gpio load i2c") != 0)
    throw runtime_error("Could not load I2C over gpio command.");

  // I2C address of IMU-3000 ---
  ros::NodeHandle n("~");
  if (n.hasParam("i2c_address"))
  {
    int paramAddr;
    n.getParam("i2c_address", paramAddr);
    address = (uint8_t) paramAddr;
    ROS_INFO("I2C address of IMU-3000 set to 0x%02X.", address);
  } 
  else 
  {
    ROS_INFO("I2C address of IMU-3000 set to default 0x%02X.", address);
  }

  // init I2C
  fd = wiringPiI2CSetup(address);
  if (fd == -1)
    throw runtime_error("I2C setup failed.");

  // configure IMU-3000 according parameters
  applyParameters();
}

/**
 * @brief Entry point of this node.
 */
int main (int argc, char** argv)
{
  try
  {
    // inits for ROS
    ros::init(argc, argv, "imu3000");
    if (ros::this_node::getNamespace() == "/")
      ROS_WARN("Started in the global namespace.");

    // init IMU3000
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

    ROS_INFO("IMU-3000 connected.");
    
    // ROS communication
    ros::NodeHandle n("~");
    ros::Publisher publisher = n.advertise<geometry_msgs::Vector3Stamped>("angular_velocity", 1);
    geometry_msgs::Vector3Stamped msg;
    msg.header.frame_id = frame_id;

    ros::Rate loop_rate(pub_rate);
    loop_rate.sleep();

    int count = 0;
    while (ros::ok())
    {
      // get new measurement data
      int16_t gyro_x, gyro_y, gyro_z;
      gyro_x = wiringPiI2CReadReg8 (fd, GYRO_XOUT_H) << 8;
      gyro_x = gyro_x | wiringPiI2CReadReg8 (fd, GYRO_XOUT_L);
      gyro_y = wiringPiI2CReadReg8 (fd, GYRO_YOUT_H) << 8;
      gyro_y = gyro_y | wiringPiI2CReadReg8 (fd, GYRO_YOUT_L);
      gyro_z = wiringPiI2CReadReg8 (fd, GYRO_ZOUT_H) << 8;
      gyro_z = gyro_z | wiringPiI2CReadReg8 (fd, GYRO_ZOUT_L);

      // convert to radians per second
      double x, y, z;
      x = COUNTS_TO_RPS(gyro_x,range) - offs_x;
      y = COUNTS_TO_RPS(gyro_y,range) - offs_y;
      z = COUNTS_TO_RPS(gyro_z,range) - offs_z;

      // fill message and publish
      msg.header.seq = count;
      msg.header.stamp = ros::Time::now();
      msg.vector.x = x;
      msg.vector.y = y;
      msg.vector.z = z;

      publisher.publish(msg);
      count++;

      ROS_DEBUG_STREAM("gyro  x , y , z : "
		       << gyro_x << " , "
		       << gyro_y << " , "
		       << gyro_z);

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

#undef PI

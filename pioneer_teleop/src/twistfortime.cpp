// cpp includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Pioneer includes
#include "pioneer3.hpp"

using namespace std;

#define MSG_BUFFER_SIZE			1000

#define ERR_PARSE			-1
#define ERR_NOLINES			-2

// Forward declarations.
int getTwistFromConfigurationfile(string line, geometry_msgs::Twist *twist, ros::Duration *duration);

/**
 * This node publishes to cmd_vel to steer the robot according to a
 * configuration file.
 */
int main(int argc, char **argv)
{
  ifstream cfgFile;

  // Initialize for ROS.
  ros::init(argc, argv, "pioneer_teleop");

  // ROS specific arguments are handled and removed by ros::init(..),
  // f.e. remapping arguments added by roslaunch.

  // Check arguments for filename.
  if (argc != 2) 
  {
    ROS_ERROR("No configuration file (holds twist-time messages to steer the robot) specified.");
    return 1;			// exit
  } 
  else 
  {
    ROS_INFO("Specified configuration file will be used for controlling the robot.");

    // Get configuration filename.
    const char* filename = argv[1];

    // Open configuration file.
    cfgFile.open(filename, ios::in);     // open the file
    if (!cfgFile.is_open())
    {
      ROS_ERROR("Cannot open configuration file (absolute path required!).");
      return 1;                   // exit if file not found
    }
  }

  // Create main access point to communicate with the ROS system.
  ros::NodeHandle n;

  // Tell ROS that we want to publish on the topic cmd_vel (for steering the robot).
  ros::Publisher pub_cmdVel = n.advertise < geometry_msgs::Twist > ("cmd_vel", MSG_BUFFER_SIZE);

  // Wait a little bit to be sure that the connection is established.
  ros::Duration durationToMove(0.5);
  durationToMove.sleep();
  durationToMove.sec = 0;	// reset duration to move
  durationToMove.nsec = 0;

  // Instantiate the message object.
  geometry_msgs::Twist twist;

  while (ros::ok())
  {
    // Get line from the configuration file.
    string line;
    if (getline(cfgFile, line)) {
      // Parse line.
      int err = getTwistFromConfigurationfile(line, &twist, &durationToMove);

      if (err == ERR_PARSE)
	continue;	// rerun this loop (this line is overjumped)
    } else {
      // e.g. ERR_NOLINES
      cfgFile.close();
      break;		// leave loop
    }

    // Send message.
    pub_cmdVel.publish(twist);
    ROS_DEBUG("%.2f m/s, %.2f degree, %.2f s", twist.linear.x, twist.angular.z, durationToMove.toSec());

    // Handle callbacks if any.
    ros::spinOnce();

    // Sleep the specified duration until next change of twist.
    durationToMove.sleep();
  }

  // Send "stop" for robot.
  twist.linear.x = 0;
  twist.angular.z = 0;
  pub_cmdVel.publish(twist);
  ROS_INFO("Robot stopped.");

  // Close connections.
  pub_cmdVel.shutdown();
  ROS_INFO("Closed connection.");

  return 0;
}

int getTwistFromConfigurationfile(string line, geometry_msgs::Twist *twist, ros::Duration *duration)
{
  double speed;        //< Speed to move the robot in m/s.
  double angle;        //< Angle to move the robot in degree.
  double time;         //< Duration to move like so in seconds.

  // Move the robot as long as there are lines in the configuration file.
  static int cntLine = 0;

  // there is a line left to parse
  cntLine++;
  stringstream ss(line);
  ss >> speed >> angle >> time;

  // ignore lines with error
  if (ss.fail())
  {
    ROS_INFO("Parse error in line %d (empty line?). This line will be ignored.", cntLine);
    ss.clear();
    return ERR_PARSE;
  }

  // Some plausible checks.
  if (speed < 0) speed = 0;
  if (speed > SPEED_MAX) speed = SPEED_MAX;

  // Fill the message with data.
  twist->linear.x = speed;
  twist->angular.z = angle;

  // Specify duration of movement.
  duration->sec = (int) time;
  duration->nsec = (time - (int)time)*1000000000;

  return 0;
}

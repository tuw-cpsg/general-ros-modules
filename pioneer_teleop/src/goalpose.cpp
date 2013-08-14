// cpp includes
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"	// type of goal pose

// Pioneer includes
#include "pioneer3.hpp"

using namespace std;

#define MSG_BUFFER_SIZE			1000

// #define ERR_PARSE			-1
// #define ERR_NOLINES			-2

enum Mode {DIRECT, INTERACTIVE, CONFIGFILE};

// Forward declarations.
void printUsage(void);
geometry_msgs::PoseStamped getGoal(float x, float y, float w);

/**
 * This node publishes a goal pose for a navigation node.
 */
int main(int argc, char **argv)
{
  Mode mode = DIRECT;
  float x,y,w;		// goal pose parameter
  // ifstream cfgFile;

  // Initialize for ROS.
  ros::init(argc, argv, "pioneer_teleop");

  // ROS specific arguments are handled and removed by ros::init(..),
  // f.e. remapping arguments added by roslaunch.

  // Check arguments for coordinates or mode.
  if (argc == 4) {
    // one goal pose is entered with xy-coordinates and rotation w
    ROS_INFO("A single goal pose will be published.");
    mode = DIRECT;

    // Get values from the arguments.
    string strX = argv[1];
    string strY = argv[2];
    string strW = argv[3];

    // Convert arguments to float.
    try {
      stringstream strStream;
      strStream << strX + " " + strY + " " + strW;
      strStream >> x >> y >> w;
    } catch(...) {
      ROS_ERROR("Invalid arguments. This node will exit.");
      printUsage();
      return 1;
    }
  } else if (argc == 2) {
    string option = argv[1];

    if (option.compare("-i") == 0) {
      ROS_INFO("Interactive mode started.");
      mode = INTERACTIVE;
    } else {
      ROS_ERROR("Invalid argument. This node will exit.");
      printUsage();
      return 1;
    }
  } else {
    ROS_ERROR("Invalid number of arguments. This node will exit.");
    printUsage();
    return 1;
  } 

  // Create main access point to communicate with the ROS system.
  ros::NodeHandle n;

  // Tell ROS that we want to publish on the topic  (for steering the robot).
  ros::Publisher pub_goal = n.advertise < geometry_msgs::PoseStamped > ("goal_pose", MSG_BUFFER_SIZE);

  // Instantiate the message object.
  geometry_msgs::PoseStamped goal;

  switch (mode) {
  case DIRECT:
    // Send a single goal to the robot to move.
    goal = getGoal(x,y,w);
    pub_goal.publish(goal);
    ROS_INFO("Send goal (x,y,w): %.2f, %.2f, %.2f", x, y, w);
    break;
  case INTERACTIVE:
    while (ros::ok())
    {
      cout << "x y w: ";
      cin >> x >> y >> w;		// attention! blocking!

      // Send a goal to the robot to move.
      goal = getGoal(x,y,w);
      pub_goal.publish(goal);
      ROS_INFO("Send goal (x,y,w): %.2f, %.2f, %.2f", x, y, w);

      // Handle callbacks if any.
      ros::spinOnce();
    }
    break;
  case CONFIGFILE:
    // TODO
    break;
  }

  // Close connections.
  pub_goal.shutdown();
  ROS_INFO("Closed connection.");

  return 0;
}

void printUsage(void) 
{
  cout << "usage: goalpose x y | configfile";
}

geometry_msgs::PoseStamped getGoal(float x, float y, float w)
{
  geometry_msgs::PoseStamped goal;

  goal.header.frame_id = "base_link";
  goal.header.stamp = ros::Time::now();

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.orientation.w = w;
  
  return goal;
}

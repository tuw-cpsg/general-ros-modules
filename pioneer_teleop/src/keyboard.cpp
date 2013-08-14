// c includes (for terminal)
#include <stdio.h>
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Pioneer includes
#include "pioneer3.hpp"

using namespace std;

#define MSG_BUFFER_SIZE			1000

// Forward declarations.
int getTwistFromKeyboard(geometry_msgs::Twist *twist);

/**
 * This node publishes to cmd_vel to steer the robot according to a
 * key pressed in the terminal window this application is running.
 */
int main(int argc, char **argv)
{
  static struct termios oldt, newt;

  // Disable buffering (till ENTER is pressed) and echo of terminal.
  tcgetattr( STDIN_FILENO, &oldt);		// get the parameters of the current terminal
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);	// change attributes immediately

  // Initialize for ROS.
  ros::init(argc, argv, "pioneer_teleop");

  // ROS specific arguments are handled and removed by ros::init(..),
  // f.e. remapping arguments added by roslaunch.
  // no arguments to parse
  ROS_INFO("This node uses blocks when waiting for a key-press. So when killing \
	     it, press any key to quit this process regurarly.");

  // Create main access point to communicate with the ROS system.
  ros::NodeHandle n;

  // Tell ROS that we want to publish on the topic cmd_vel (for steering the robot).
  ros::Publisher pub_cmdVel = n.advertise < geometry_msgs::Twist > ("cmd_vel", MSG_BUFFER_SIZE);

  // Wait a little bit to be sure that the connection is established.
  ros::Duration duration(0.5);
  duration.sleep();

  // Instantiate the message object.
  geometry_msgs::Twist twist;

  while (ros::ok())
  {
    if (getTwistFromKeyboard(&twist) != 0)	// blocking!
      break;

    // Send message.
    pub_cmdVel.publish(twist);
    ROS_DEBUG("%.2f m/s, %.2f degree", twist.linear.x, twist.angular.z);

    // Handle callbacks if any.
    ros::spinOnce();
  }

  // Send "stop" for robot.
  twist.linear.x = 0;
  twist.angular.z = 0;
  pub_cmdVel.publish(twist);
  ROS_INFO("Robot stopped.");

  // Close connections.
  pub_cmdVel.shutdown();
  ROS_INFO("Closed connection.");

  // Restore old terminal settings.
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);

  return 0;
}

/**
 * Waits on a key pressed and sets the twist according to this key.
 */
int getTwistFromKeyboard(geometry_msgs::Twist *twist)
{
  static float speed = SPEED_MIN;
  char keyPressed;
  
  //cin >> keyPressed;		// cin cannot read space
  keyPressed = getchar();	// blocking!!!

  switch (keyPressed)
  {
    case ' ':	// space
      // stop
      twist->linear.x = 0;
      twist->angular.z = 0;
      ROS_INFO("stop");
      break;
    case 'a':
      // left
      if (speed < SPEED_MIN)
	speed = SPEED_MIN;

      twist->angular.z = speed * SCALE_ROTATION;
      ROS_INFO("left");
      break;
    case 'd':
      // right
      if (speed < SPEED_MIN)
	speed = SPEED_MIN;

      twist->angular.z = -speed * SCALE_ROTATION;
      ROS_INFO("right");
      break;
    case 's':
      // backward
      if (speed < SPEED_MIN)
	speed = SPEED_MIN;

      twist->linear.x = -speed * SCALE_TRANSLATION;
      twist->angular.z = 0;
      ROS_INFO("backward");
      break;
    case 'w':
      // forward
      if (speed < SPEED_MIN)
	speed = SPEED_MIN;

      twist->linear.x = speed * SCALE_TRANSLATION;
      twist->angular.z = 0;
      ROS_INFO("forward");
      break;
    case 'i':
      // increase speed
      if (speed < SPEED_MAX)
      {
        speed += SPEED_STEP;
        ROS_INFO("speed ++ to %.2f", speed);
      }
      break;
    case 'k':
      // decrease speed
      if (speed > SPEED_MIN)
      {
        speed -= SPEED_STEP;
        ROS_INFO("speed -- to %.2f", speed);
      }
      break;
    case 'q':
      // quit
      ROS_INFO("quit");
      return 1;
  }
  
  return 0;
}

/**
 * @file keyboard.cpp
 * @date 28.06.2013
 * @author Denise Ratasich
 * 
 * @brief ROS node reading from stdin to steer the robot via keyboard.
 *
 * A new speed is only published when a key is pressed. Note reading
 * vom stdin (console) is blocking.
 *
 * - subscribe: none
 * - publish: ~cmd_vel [geometry_msgs/Twist]
 *
 * parameters
 * - ~pub_period_ms: The publishing period of cmd_vel in ms. Default
 *   is set to 500ms.
 */

// c includes (for terminal, threading)
#include <stdio.h>
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO
#include <boost/thread.hpp>

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Pioneer includes
#include "pioneer3.hpp"

using namespace std;

#define MSG_BUFFER_SIZE			1

// Parameters.
int pub_period_ms = 500;

/**
 * @brief Guards a twist object and an additional running flag for the
 * keyboard thread, i.e., ensures mutual exclusion of the variables to
 * exchange.
 */
class MutexedData {
  boost::mutex mtx_;			// for mutual exclusion
  geometry_msgs::Twist twist_;		// the data to send
  ros::Publisher pub_cmdVel_;		// the publisher (thread is also able to publish)
  int running_;
public:
  void init(ros::NodeHandle& n) {
    twist_.linear.x = 0;
    twist_.angular.z = 0;
    pub_cmdVel_ = n.advertise < geometry_msgs::Twist > ("cmd_vel", MSG_BUFFER_SIZE);
    running_ = 0;
  }
  void shutdown() {
    pub_cmdVel_.shutdown();
  }
  void setTwist(geometry_msgs::Twist& twist) {
    mtx_.lock();
    twist_.linear.x = twist.linear.x;
    twist_.angular.z = twist.angular.z;
    mtx_.unlock();
  }
  void setTwist(float lin, float ang) {
    mtx_.lock();
    twist_.linear.x = lin;
    twist_.angular.z = ang;
    mtx_.unlock();
  }
  geometry_msgs::Twist getTwist() {
    geometry_msgs::Twist twist;
    mtx_.lock();
    twist.linear.x = twist_.linear.x;
    twist.angular.z = twist_.angular.z;
    mtx_.unlock();
    return twist;
  }
  void publish() {
    mtx_.lock();
    pub_cmdVel_.publish(twist_);
    ROS_DEBUG("%.2f m/s, %.2f degree", twist_.linear.x, twist_.angular.z);
    mtx_.unlock();
  }
  void publish(float lin, float ang) {
    mtx_.lock();
    twist_.linear.x = lin;
    twist_.angular.z = ang;
    pub_cmdVel_.publish(twist_);
    ROS_DEBUG("%.2f m/s, %.2f degree", twist_.linear.x, twist_.angular.z);
    mtx_.unlock();
  }
  void setRunning(int running) {
    mtx_.lock();
    running_ = running;
    mtx_.unlock();
  }
  int isRunning() {
    int r;
    mtx_.lock();
    r = running_;
    mtx_.unlock();
    return r;
  }
};

// Instantiate the guarded twist object.
MutexedData mtxData;

// Forward declarations.
void applyParameters(ros::NodeHandle& n);
void getTwistFromKeyboard();

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
  if (ros::this_node::getNamespace() == "/")
    ROS_WARN("Started in the global namespace.");

  // Create main access point to communicate with the ROS system.
  ros::NodeHandle n("~");
  
  // Apply parameters from parameter server.
  applyParameters(n);

  // Note, how to terminate this node!
  ROS_WARN("This node can only be terminated by pressing 'q'.");

  // Initialize data and tell ROS that we want to publish on the topic
  // cmd_vel (for steering the robot).
  mtxData.init(n);

  // Wait a little bit to be sure that the connection is established.
  ros::Duration duration(0.5);
  duration.sleep();
  duration.sec = 0;
  duration.nsec = pub_period_ms * 1e6;

  // Start thread to read from keyboard, updates mtxData if a key is
  // pressed.
  boost::thread keyboardThread(getTwistFromKeyboard);
  mtxData.setRunning(1);	// set running flag of the thread

  while (ros::ok() && mtxData.isRunning())
  {
    // Send message periodically.
    mtxData.publish();

    // Handle callbacks if any.
    ros::spinOnce();

    // Timeout.
    duration.sleep();
  }

  if (mtxData.isRunning())
    ROS_ERROR("Ctrl-C received. Press 'q' to quit the node!");
  keyboardThread.join();

  // Send "stop" for robot.
  mtxData.publish(0,0);
  ROS_INFO("Robot stopped.");

  // Close connections.
  mtxData.shutdown();
  ROS_INFO("Closed connection.");

  // Restore old terminal settings.
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);

  return 0;
}

/**
 * @brief Fetches available params from the parameter server and
 * configures this node.
 */
void applyParameters(ros::NodeHandle& n)
{
  // ROS params ---
  if (n.hasParam("pub_period_ms")) {
    n.getParam("pub_period_ms", pub_period_ms);
    ROS_INFO("Publishing period set to %d ms.", pub_period_ms);
  } else {
    ROS_INFO("Publishing period set to default %d ms.", pub_period_ms);
  }
}

/**
 * @brief Waits on a key pressed and sets the twist according to this
 * key.
 */
void getTwistFromKeyboard()
{
  float speed_lin = 0;
  float speed_ang = 0;
  char keyPressed;
  bool run = 1;

  while(run)
  {
    //cin >> keyPressed;		// cin cannot read space
    keyPressed = getchar();	// blocking!!!

    switch (keyPressed)
    {
    case ' ':	// space
      // stop
      speed_lin = 0;
      speed_ang = 0;
      ROS_INFO("stop");
      break;
    case 'a':
      // left
      if (speed_ang < SPEED_MAX)
	speed_ang += SPEED_STEP;
      break;
    case 'd':
      // right
      if (speed_ang > -SPEED_MAX)
	speed_ang -= SPEED_STEP;
      break;
    case 's':
      // backward
      if (speed_lin > -SPEED_MAX)
	speed_lin -= SPEED_STEP;

      if (speed_ang != 0) {
	speed_ang = 0;
	ROS_INFO("straight backward");
      }
      break;
    case 'w':
      // forward
      if (speed_lin < SPEED_MAX)
	speed_lin += SPEED_STEP;

      if (speed_ang != 0) {
	speed_ang = 0;
	ROS_INFO("straight forward");
      }
      break;
    case 'q':
      // quit
      ROS_INFO("quit");
      mtxData.setRunning(0);	
      run = 0;			// leave loop and terminate this thread
    }

    mtxData.publish(speed_lin * SCALE_TRANSLATION, speed_ang * SCALE_ROTATION);
  }
}

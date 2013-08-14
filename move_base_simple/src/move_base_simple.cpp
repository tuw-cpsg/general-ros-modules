/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: GonÃ§alo Cabrita on 27/08/2012
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <angles/angles.h>

// Simple Navigation States
typedef enum _SimpleNavigationState {
    
    SN_STOPPED = 1,
    SN_MOVING = 2,
    SN_ROTATING = 3,
    SN_MOVING_AS = 4,
    SN_ROTATING_AS = 5
    
} SimpleNavigationState;

// Simple Navigation State
SimpleNavigationState state;

// Global frame_id
std::string global_frame_id;

// Target position
geometry_msgs::PoseStamped goal;
// Robot odometry
nav_msgs::Odometry odom;

bool rotate_in_place;

void goalReceived(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;

    if(rotate_in_place) state = SN_ROTATING;
    else state = SN_MOVING;
}

void odomReceived(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_simple");
    
    ROS_INFO("Move Base Simple for ROS");
    
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    
    // Parameters
    double rate;
    double in_place_angular_velocity;
    double max_linear_velocity;
    double min_linear_velocity;
    double alpha;
    double attraction_coefficient;
    double repulsion_coefficient;
    double goal_tolerance;
    double angular_threshold;

    bool visualization;
    
    pn.param("rate", rate, 3.0);
    pn.param("in_place_angular_velocity", in_place_angular_velocity, 3.0);
    pn.param("max_linear_velocity", max_linear_velocity, 0.2);
    pn.param("min_linear_velocity", min_linear_velocity, 0.05);
    pn.param("alpha", alpha, 0.5);
    pn.param("attraction_coefficient", attraction_coefficient, 0.5);
    pn.param("goal_tolerance", goal_tolerance, 0.10);
    pn.param("angular_threshold", angular_threshold, 0.4);
    pn.param("visualization", visualization, false);

    if(angular_threshold == 0.0)
    {
	rotate_in_place = false;
	ROS_INFO("MoveBase Simple -- Not using in-place rotations.");
    }
    else
    {
    	rotate_in_place = true;
	ROS_INFO("MoveBase Simple -- Using in-place rotations.");
    }

    pn.param<std::string>("global_frame_id", global_frame_id, "/base_link");
    
    ROS_INFO("MoveBase Simple -- Using %s as the global frame.", global_frame_id.c_str());
	
    // Making all the publishing and subscriptions...
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 20, odomReceived);
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>("goal_pose", 10, goalReceived);
    
    state = SN_STOPPED;
    
    // Main Loop
    ros::Rate r(rate);
    while(n.ok())
    {
        double linear_velocity = 0.0;
        double angular_velocity = 0.0;
        
        double current_orientation = tf::getYaw(odom.pose.pose.orientation);
        double current_x = odom.pose.pose.position.x;
        double current_y = odom.pose.pose.position.y;
        
	// If we reached our target position 
        if((state == SN_MOVING || state == SN_MOVING_AS || state == SN_ROTATING || state == SN_ROTATING_AS) && sqrt(pow(current_x-goal.pose.position.x,2)+pow(current_y-goal.pose.position.y,2)) < goal_tolerance)
        {
            state = SN_STOPPED;
            linear_velocity = 0.0;
            angular_velocity = 0.0;
        }

        // If we are moving...
        if(state == SN_MOVING || state == SN_MOVING_AS || state == SN_ROTATING || state == SN_ROTATING_AS)
        {
            double G_attr_x = -attraction_coefficient*(current_x-goal.pose.position.x);
            double G_attr_y = -attraction_coefficient*(current_y-goal.pose.position.y);
            
            double target_orientation = atan2(G_attr_y, G_attr_x);
            
            linear_velocity = sqrt(G_attr_x*G_attr_x + G_attr_y*G_attr_y);
            if(fabs(linear_velocity) > max_linear_velocity) linear_velocity = (linear_velocity > 0 ? max_linear_velocity : -max_linear_velocity);
            if(fabs(linear_velocity) < min_linear_velocity) linear_velocity = (linear_velocity > 0 ? min_linear_velocity : -min_linear_velocity);

	    angular_velocity = -alpha*(angles::shortest_angular_distance(target_orientation, current_orientation));

	    // If we intend to rotate before moving forward...
	    if(state == SN_ROTATING || state == SN_ROTATING_AS)
	    {
	    	linear_velocity = 0.0;
   		angular_velocity = (angular_velocity < 0 ? -in_place_angular_velocity : in_place_angular_velocity);
	
	    	if(fabs(angles::shortest_angular_distance(current_orientation, target_orientation)) < angular_threshold)
	    	{
            		angular_velocity = 0.0;
	    		if(state == SN_ROTATING) state = SN_MOVING;
	    		else if(state == SN_ROTATING_AS) state = SN_MOVING_AS;
	    	}
	    }
        }
        
        // Send the new velocities to the robot...
        geometry_msgs::Twist cmd_vel;
        
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
        
        cmd_vel_pub.publish(cmd_vel);
        
        ros::spinOnce();
	r.sleep();
    }

  	return(0);
}

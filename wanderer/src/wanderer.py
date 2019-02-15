#! /usr/bin/env python
"""Wanderer.

__author__ Denise Ratasich
__date__ 2018-12-05

Note, can get stuck (rotating) in dynamic environments.

"""

import roslib
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


_node_name = "wanderer"
"""Name of the ROS node."""


class Wanderer(object):

    def __init__(self, timeout=0.5):
        # init state
        self.__state = 'stop'
        rospy.loginfo("{}: initialized".format(_node_name))
        # timeout for laser data
        self.__watchdog_timeout = timeout
        # params (minimum distance hysteresis)
        self.__dsafe_bot = rospy.get_param('~dsafe_bottom', 0.5)
        self.__dsafe_top = rospy.get_param('~dsafe_top', 0.6)
        rospy.loginfo("{}: safe bot = {}, safe top = {}".format(_node_name,
            self.__dsafe_bot, self.__dsafe_top))
        # publish/subscribe
        self.__sub = rospy.Subscriber('~scan', LaserScan, self.__callback_laser)
        rospy.loginfo("{}: subscribed ~scan".format(_node_name))
        self.__pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
        rospy.loginfo("{}: will publish to ~cmd_vel".format(_node_name))

    def __enter__(self):
        # init state
        self.__state = 'stop'
        # watchdog to be sure publishing cmd_vel regularly
        # in case laser fails, rover is stopped
        self.__watchdog = rospy.Timer(rospy.Duration(self.__watchdog_timeout),
                                      self.__watchdog_callback)

    def __set_output(self, state):
        # set and send cmd_vel
        msgtosend = Twist()
        # apply state
        if self.__state == 'stop':
            msgtosend.linear.x = 0.0
            msgtosend.angular.z = 0.0
        elif self.__state == 'turn left':
            msgtosend.linear.x = 0.0
            msgtosend.angular.z = 0.3
        elif self.__state == 'turn right':
            msgtosend.linear.x = 0.0
            msgtosend.angular.z = -0.3
        elif self.__state == 'forward':
            msgtosend.linear.x = 0.1
            msgtosend.angular.z = 0.0
        else:
            rospy.logerr("{}: reached wrong state: '{}'".format(_node_name,
                                                                self.__state))
            msgtosend.linear.x = 0.0
            msgtosend.angular.z = 0.0
            rospy.logwarn("{}: stop".format(_node_name))
        self.__pub.publish(msgtosend)

    def __callback_laser(self, msg):
        """Called when a message of the topic 'scan' is received."""
        dmin = min(msg.ranges)
        dmax = max(msg.ranges)  # could give dmax=inf
        dmax_angle = msg.ranges.index(dmax) * msg.angle_increment + msg.angle_min
        dmax = min(5,dmax)  # say 5m are enough (handle 'inf')
        dfront = msg.ranges[int(float(msg.angle_max - msg.angle_min)/msg.angle_increment/2)]
        rospy.logdebug("{}: received laser scan," \
                       "dfront: {:.2f}," \
                       "dmin..dmax: {:.2f}..{:.2f}" \
                       "dmax_angle: {}".format(_node_name, dfront, dmin, dmax,
                                               dmax_angle))
        self.__watchdog_restart()
        # state update
        if 'turn' in self.__state:
            # turn until we reach at least (dfront > 0.9*dmax and dmin > _dsafe_top)
            if dfront > 0.9*dmax and dmin > self.__dsafe_top:
                rospy.loginfo("forward")
                self.__state = 'forward'
        elif self.__state == 'stop':
            # got first laser scan, start moving
            self.__state = 'turn left'
        elif self.__state == 'forward':
            if dmin < self.__dsafe_bot and dmax < self.__dsafe_bot:
                rospy.logwarn("surrounded by obstacles -> turn")
                self.__state = 'turn left'
            elif dmin < self.__dsafe_top:
                if dmax_angle < 0:
                    rospy.loginfo("turn right")
                    self.__state = 'turn right'
                else:
                    rospy.loginfo("turn left")
                    self.__state = 'turn left'
        else:
            rospy.logerr("{}: reached wrong state: '{}'".format(_node_name, self.__state))
        # output update
        self.__set_output(self.__state)

    def __watchdog_restart(self):
        # watchdog restart (unfortunately there is no start/stop)
        self.__watchdog.shutdown()
        self.__watchdog = rospy.Timer(rospy.Duration(self.__watchdog_timeout),
                                      self.__watchdog_callback)

    def __watchdog_callback(self, event):
        # no laser data for a while -> stop
        self.__state = 'stop'
        self.__set_output(self.__state)
        self.__watchdog_restart()

    def __exit__(self, exc_type, exc_value, traceback):
        self.__sub.unregister()
        self.__pub.unregister()


# main entry point of this node
if __name__ == '__main__':
    try:
        rospy.init_node(_node_name)
        with Wanderer() as w:
            rospy.spin()
        # avoid unresolved race condition during shutdown causing exception
        # https://github.com/ros/ros_comm/issues/527
        #rospy.sleep(0.5)
    except Exception as e:
        rospy.logerr("{}: node failed. {}".format(_node_name, e))
        raise
    except rospy.ROSInterruptException:
        pass

#! /usr/bin/env python
"""Emergency stop node.

__author__ Denise Ratasich
__date__ 2017-09-25

"""

import roslib
import rospy

from additional_msgs.msg import Float32Stamped
from geometry_msgs.msg import Twist


_node_name = 'emergency_stop'
"""Name of this node in the ROS system."""
_pub_cmdvel = None
"""Publisher of 'cmd_vel'."""
_stop = False
"""Stop the rover."""
_dsafe_bot = None
_dsafe_top = None
"""Minimal distance required (hysteresis)."""


def callback_dmin(msg):
    """Called when a message of the topic 'dmin' is received."""
    global _stop, _dsafe1, _dsafe2
    rospy.logdebug("received dmin: %f", msg.data)
    if msg.data < _dsafe_bot:
        if not _stop:
            rospy.logwarn("dmin falls below dsafe (%f)", _dsafe_bot)
        _stop = True
    elif msg.data > _dsafe_top:
        if _stop:
            rospy.loginfo("dmin ok again (>%f)", _dsafe_top)
        _stop = False
    else:
        # robot's dmin is near dsafe
        pass


def callback_cmdvel(msg):
    """Filters received messages of topic 'cmd_vel' w.r.t. minimum distance."""
    global _stop
    rospy.logdebug("received cmd_vel: (%f,%f)", msg.linear.x, msg.angular.z)
    msgtosend = msg
    if _stop and msg.linear.x > 0:
        msgtosend.linear.x = 0
        rospy.logdebug("reset cmd_vel(.linear.x)")
    _pub_cmdvel.publish(msg)


# main entry point of this node
if __name__ == '__main__':
    try:
        rospy.init_node(_node_name)
        # params (minimum distance hysteresis)
        _dsafe_bot = rospy.get_param('~dsafe_bottom', 0.5)
        _dsafe_top = rospy.get_param('~dsafe_top', 0.6)
        # subscribe to 'dmin'
        sub_dmin = rospy.Subscriber('~dmin', Float32Stamped, callback_dmin)
        # filter 'cmd_vel'
        sub_cmdvel = rospy.Subscriber('~cmd_vel_in', Twist, callback_cmdvel)
        _pub_cmdvel = rospy.Publisher('~cmd_vel_out', Twist, queue_size=10)
        rospy.loginfo("initialized")
        # loop over receive-send-sleep
        rospy.spin()
        # cleanup
        sub_dmin.unregister()
        sub_cmdvel.unregister()
        # avoid unresolved race condition during shutdown causing exception
        # https://github.com/ros/ros_comm/issues/527
        rospy.sleep(0.5)
    except Exception as e:
        rospy.logerr("Emergency stop node failed. %s", e)
        raise
    except rospy.ROSInterruptException:
        pass

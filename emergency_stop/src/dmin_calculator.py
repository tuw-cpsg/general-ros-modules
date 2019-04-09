#! /usr/bin/env python
"""Minimum distance calculator node.

__author__ Denise Ratasich
__date__ 2017-09-25

"""

import roslib
import rospy

from sensor_msgs.msg import LaserScan
from additional_msgs.msg import Float32Stamped


_node_name = 'dmin_calculator'
"""Name of this node in the ROS system."""
_pub = None
"""Publisher of 'dmin'."""


def callback_laser(msg):
    """Called when a message of the topic 'scan' is received."""
    global _pub
    ranges = msg.ranges
    if len(msg.intensities) > 0:
        ranges = [r for i, r in enumerate(msg.ranges) if msg.intensities[i] > 0]
    dmin = min(ranges)
    rospy.logdebug("received laser scan, dmin: %f", dmin)
    msgtosend = Float32Stamped()
    msgtosend.header.stamp = msg.header.stamp  # keep timestamp of laser scan
    msgtosend.data = dmin
    _pub.publish(msgtosend)


# main entry point of this node
if __name__ == '__main__':
    try:
        rospy.init_node(_node_name)
        # subscribe to 'dmin'
        sub = rospy.Subscriber('~scan', LaserScan, callback_laser)
        _pub = rospy.Publisher('~dmin', Float32Stamped, queue_size=10)
        rospy.loginfo("initialized")
        # loop over receive-send-sleep
        rospy.spin()
        # cleanup
        sub.unregister()
        _pub.unregister()
        # avoid unresolved race condition during shutdown causing exception
        # https://github.com/ros/ros_comm/issues/527
        rospy.sleep(0.5)
    except Exception as e:
        rospy.logerr("dmin calculator node failed. %s", e)
        raise
    except rospy.ROSInterruptException:
        pass

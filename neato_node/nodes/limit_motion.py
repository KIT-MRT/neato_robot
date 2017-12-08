#!/usr/bin/env python
"""
ROS node for limit motions

This nodes limits the motion of the robot. The motion is limited to only linear or only angular motion and do not
publish two different motions between one odometry message (except stop motion) to avoid errors in odometry message.
"""
import roslib
roslib.load_manifest('neato_node')
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

odometry_called = False

def command_callback(data):
    """callback when new driving command

    Args:
        data (LaserScan)
    """
    global odometry_called
    if data.twist.linear.x == 0 \
            and data.twist.linear.y == 0 \
            and data.twist.linear.z == 0 \
            and data.twist.angular.x == 0 \
            and data.twist.angular.y == 0 \
            and data.twist.angular.z == 0:
        command_publisher.publish(data) 
        odometry_called = False
        return
    elif data.twist.angular.x == 0 \
            and data.twist.angular.y == 0 \
            and data.twist.angular.z == 0:
        pass
    else:
        data.twist.linear.x = 0
        data.twist.linear.y = 0
        data.twist.linear.z = 0
    if odometry_called:
        command_publisher.publish(data) 
        odometry_called = False

def odometry_callback(data):
    """callback when new odometry data is available

    Args:
        data (Odometry)
    """
    global odometry_called
    odometry_called = True


if __name__ == '__main__':
    rospy.init_node('limit_motion')

    rospy.Subscriber("vel_cmd", Twist, command_callback, queue_size=1)
    rospy.Subscriber("odom", Odometry, odometry_callback, queue_size=1)
    command_publisher = rospy.Publisher("limited_vel_cmd", Twist, queue_size=1)

    rospy.spin()


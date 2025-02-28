#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import tiago_pkg.utils as Utils

Utils.call_service('wheel_left_joint', 'set_velocity', 0)
Utils.call_service('wheel_right_joint', 'set_velocity', 0)
Utils.call_service('wheel_left_joint', 'set_position', float('inf'))
Utils.call_service('wheel_right_joint', 'set_position', float('inf'))

def readVelocity(data):
    v = data.linear.x
    w = data.angular.z
    vl = (2*v - w*Utils.wheel_distance)/(2*Utils.wheel_radius)
    vr = (2*v + w*Utils.wheel_distance)/(2*Utils.wheel_radius)
    Utils.call_service('wheel_left_joint', 'set_velocity', vl)
    Utils.call_service('wheel_right_joint', 'set_velocity', vr)

rospy.init_node("cmd_vel", anonymous=True)
my_sub = rospy.Subscriber("/cmd_vel", Twist, readVelocity, queue_size=1)
rospy.spin()
#!/usr/bin/env python3

import rospy
import rosservice
import math

# Caratteristiche robot
robot_name = 'Tiago'
arm_payload = 3 #kg
arm_reach = 0.87 #m
torso_lift = 0.35 #m
max_speed = 10.1523 #rad/s
laser_range = 10 #m
height_min = 1.10 #m
height_max = 1.45 #m
weight = 70 #kg
footprint = 0.54 #m
wheel_distance = 0.4044 #m
wheel_radius = 0.0985 #m
wheel_circum = 2 * math.pi * wheel_radius #m

timestep = 32

def call_service(device_name, service_name, *args):
	service_string = "/%s/%s/%s" % (robot_name, device_name, service_name)
	rospy.loginfo(service_string)
	rospy.wait_for_service(service_string)
	try:
		service = rospy.ServiceProxy(service_string, rosservice.get_service_class_by_name(service_string))
		response = service(*args)
		rospy.loginfo("Service %s called" % service_string)
		return response
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: %s" % e)

#!/usr/bin/env python3

import math
import rospy
import tf
from nav_msgs.msg import Odometry
import tiago_pkg.utils as Utils
from webots_ros.msg import Float64Stamped
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3, Pose

rospy.init_node('odometry_publisher', anonymous=True)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
Utils.call_service('wheel_left_joint_sensor', 'enable', True)
Utils.call_service('wheel_right_joint_sensor', 'enable', True)
last_time = rospy.Time()
#-------------------
ps_value = [0,0]
last_value = [0,0]
x = 0
y = 0
th = 0
#-------------------
r = rospy.Rate(1)

def readLeft(data):
    global ps_value
    ps_value[0] = data.data

def readRight(data):
    global ps_value
    ps_value[1] = data.data

sub_left = rospy.Subscriber(Utils.robot_name + "/wheel_left_joint_sensor/value", Float64Stamped, readLeft)
sub_right = rospy.Subscriber(Utils.robot_name + "/wheel_right_joint_sensor/value", Float64Stamped, readRight)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()   
    dt = (current_time - last_time).to_sec()
    
    if abs(ps_value[0] - last_value[0]) < 0.001:
        ps_value[0] = last_value[0]
    if abs(ps_value[1] - last_value[1]) < 0.001:
        ps_value[1] = last_value[1]
        
    vl = (ps_value[0] - last_value[0]) / dt
    vr = (ps_value[1] - last_value[1]) / dt
    
    w = (vr - vl) * (Utils.wheel_radius / Utils.wheel_distance)
    th += w * dt
    v = (vr + vl)/2

    vx = v * math.cos(th)
    vy = v * math.sin(th)
    x += vx * dt * Utils.wheel_radius
    y += vy * dt * Utils.wheel_radius

    for i in range(2):
        last_value[i] = ps_value[i]

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, w))
    
    odom_pub.publish(odom)
    
    last_time = current_time
    r.sleep()

#!/usr/bin/env python3

import rospy
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from geometry_msgs.msg import Quaternion
import tiago_pkg.utils as Utils
from webots_ros.msg import BoolStamped

pedestrian = False
flag = False
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
goalx = goaly = 0

def readDetect(data):
    global pedestrian
    pedestrian = data.data

def callback(msg):
    global flag
    if pedestrian:
        client.cancel_goal()
        flag = True
        
def first_try(msg):
    global flag
    x_act = msg.base_position.pose.position.x
    y_act = msg.base_position.pose.position.y
    
    if pedestrian:
        if (abs(goalx - x_act) < 1) and (abs(goaly - y_act) < 1):
            client.cancel_goal()
            flag = True 

def museum_move(x, y, theta):
    global flag, goalx, goaly
    flag = False 
    my_sub = rospy.Subscriber("/pedestrian_detect", BoolStamped, readDetect)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    quat_goal = tf.transformations.quaternion_from_euler(0, 0, theta)
    quat_goal = Quaternion(*quat_goal)
    goal.target_pose.pose.orientation.x = quat_goal.x
    goal.target_pose.pose.orientation.y = quat_goal.y
    goal.target_pose.pose.orientation.z = quat_goal.z
    goal.target_pose.pose.orientation.w = quat_goal.w
    goalx = x
    goaly = y
    client.send_goal(goal, feedback_cb=first_try)
    wait = client.wait_for_result()

    if not wait:
        pass
    else:
        if client.get_result():
            first_rotate = move_feedback(x,y,theta+math.pi/4,function=callback)
            if not first_rotate:
                second_rotate = move_feedback(x,y,theta-math.pi/4,function=callback)
                if second_rotate:
                    return second_rotate
                else:
                    return False
            else:
                return first_rotate
        elif flag:
            return True
        else:
            return False
            
def move_feedback(x,y,theta,function=None):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    quat_goal = tf.transformations.quaternion_from_euler(0, 0, theta)
    quat_goal = Quaternion(*quat_goal)
    goal.target_pose.pose.orientation.x = quat_goal.x
    goal.target_pose.pose.orientation.y = quat_goal.y
    goal.target_pose.pose.orientation.z = quat_goal.z
    goal.target_pose.pose.orientation.w = quat_goal.w
    client.send_goal(goal, feedback_cb=function)
    wait = client.wait_for_result()
    
    if not wait:
        pass
    return flag

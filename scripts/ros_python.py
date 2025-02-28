#!/usr/bin/env python3

import rospy
import time
import json
import os
import tiago_pkg.utils as Utils
from tiago_pkg.lidar import Lidar
from tiago_pkg.speaker import Speaker
from tiago_pkg.move_base import museum_move

rospy.init_node('my_robot', anonymous=True)

HOST = os.popen('whoami').read().strip('\n')
PATH = f"/home/{HOST}/catkin_ws/src/webots_ros/config/"

f = open(f"{PATH}info.json", "r")
data = json.load(f)
f.close()

if __name__ == '__main__':
    try:
        lidar = Lidar()
        speaker = Speaker()
        Utils.call_service('torso_lift_joint', 'set_position', 0.25) # max=0.35
        time.sleep(3)
        i = 0
        while not rospy.is_shutdown():
            if(not speaker.is_speaking()):
                result = museum_move(data['goals'][i]['x'], data['goals'][i]['y'], data['goals'][i]['theta'])
                if result:
                    speaker.speak(data['paintings'][i]['descrizione'])
                i += 1
                i %= len(data['goals'])
    except rospy.ROSInterruptException:
        pass

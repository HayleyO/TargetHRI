#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi


if __name__ == '__main__':
    rospy.init_node('Testing_Motion', anonymous=True)
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    move_cmd = Twist()
    
    duration = 5
    linear_speed = 0.2
    angular_speed = pi*4/4
    r = rospy.Rate(10)

    parsing = True
    while parsing:
        input_dir = raw_input("Give a direction: ")
        if input_dir.lower() in ["up", "u"]:
            print("In Up")
            move_cmd.linear.x = -linear_speed
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            rospy.sleep(duration)
        elif input_dir.lower() in ["down", "d"]:
            move_cmd.linear.x = linear_speed
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            rospy.sleep(duration)
        elif input_dir.lower() in ["right", "r"]:
            move_cmd.angular.z = -angular_speed
            move_cmd.linear.x = 0
            cmd_vel.publish(move_cmd)
            rospy.sleep(duration)
        elif input_dir.lower() in ["left", "l"]:
            move_cmd.angular.z = angular_speed
            move_cmd.linear.x = 0
            cmd_vel.publish(move_cmd)
            rospy.sleep(duration)
        elif input_dir.lower() == "x":
            parsing = False
        

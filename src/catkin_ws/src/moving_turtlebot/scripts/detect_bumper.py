#!/usr/bin/env python

import rospy
from kobuki_msgs.msg import BumperEvent

def callback(data):
    print("Target has been found")
    #state: 0 = released, 1 = pressed
    #bumper: 0 = left, 1 = center, 2 = right
     
def listener():
    print("listening")
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    
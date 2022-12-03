import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    move_cmd = Twist()
    # let's go forward 
    move_cmd.linear.x = 0.5
    # let's turns
    move_cmd.angular.z = 0
    cmd_vel.publish(move_cmd)
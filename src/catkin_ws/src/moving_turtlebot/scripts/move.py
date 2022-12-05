import rospy
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    rospy.init_node('Testing_Motion', anonymous=True)
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    move_cmd = Twist()
    
    while True:
        parsing = True
        while parsing:
            input = input("Give a direction: ")
            if input.lower() in["up", "u"]:
                move_cmd.linear.x = -0.5
                move_cmd.angular.z = 0
            elif input.lower() in["down", "d"]:
                move_cmd.linear.x = 0.5
                move_cmd.angular.z = 0
            elif input.lower() in["right", "r"]:
                move_cmd.linear.y = 0.5
                move_cmd.angular.z = 0
            elif input.lower() in["left", "l"]:
                move_cmd.linear.y = -0.5
                move_cmd.angular.z = 0
        cmd_vel.publish(move_cmd)

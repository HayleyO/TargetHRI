#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

duration = 4
linear_speed = 0.4

def move_forward(x, move_cmd, cmd_vel, r):
    # for _ in range(duration):
    move_cmd.linear.x = x*linear_speed
    move_cmd.angular.z = 0
    for _ in range(duration):
    	cmd_vel.publish(move_cmd)
    	r.sleep()

def rotate(x, move_cmd, cmd_vel):
    move_cmd.angular.z = x*pi*4/4
    move_cmd.linear.x = 0
    cmd_vel.publish(move_cmd)
    rospy.sleep(duration)
 
if __name__ == '__main__':
    rospy.init_node('Testing_Motion', anonymous=True)
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    move_cmd = Twist()
    r = rospy.Rate(2)

    maze_x, maze_y = 100, 100
    grid = [['.' for _ in range(maze_y)] for i in range(maze_x)]
    cur_x, cur_y = 0, 0

    facing = [0, 1]

    parsing = True
    while parsing:
        input_dir = raw_input("Give a direction: ")
        if input_dir.lower() in ["down", "d"]:
            if cur_x-facing[0] < maze_x and cur_x-facing[0]>=0 and cur_y-facing[1]<maze_y and cur_y-facing[1]>=0:
                move_forward(-1.1, move_cmd, cmd_vel, r)
                grid[cur_x][cur_y] = '.'
                cur_x -= facing[0]
                cur_y -= facing[1]
                grid[cur_x][cur_y] = 'o'
		print(cur_x, cur_y)
		print(facing)
            else:
                print('Invalid movement')

        elif input_dir.lower() in ["up", "u"]:
            if cur_x+facing[0] < maze_x and cur_x+facing[0]>=0 and cur_y+facing[1]<maze_y and cur_y+facing[1]>=0:
                move_forward(1, move_cmd, cmd_vel, r)
                grid[cur_x][cur_y] = '.'
                cur_x += facing[0]
                cur_y += facing[1]
                grid[cur_x][cur_y] = 'o'
		print(cur_x, cur_y)
		print(facing)
            else:
                print('Invalid movement')

        elif input_dir.lower() in ["left", "l"]:
            if True:
                # Turn left
                rotate(-1.13, move_cmd, cmd_vel)
		
            if facing == [1, 0]:
		        facing = [0, 1]
            elif facing == [0, 1]:
		        facing = [-1, 0]
            elif facing == [-1, 0]:
		        facing = [0, -1]
            elif facing == [0, -1]:
		        facing = [1, 0]
        
            move_forward(1, move_cmd, cmd_vel, r)
            grid[cur_x][cur_y] = '.'
            cur_x += facing[0]
            cur_y += facing[1]
            grid[cur_x][cur_y] = 'o'
		
            print(facing)

        elif input_dir.lower() in ["right", "r"]:
            if True:
                # Turn right
                rotate(1.127, move_cmd, cmd_vel)

                if facing == [1, 0]:
                    facing = [0, -1]
                elif facing == [0, -1]:
                    facing = [-1, 0]
                elif facing == [-1, 0]:
                    facing = [0, 1]
                elif facing == [0, 1]:
                    facing = [1, 0]
            print(facing)
            move_forward(1, move_cmd, cmd_vel, r)
            grid[cur_x][cur_y] = '.'
            cur_x += facing[0]
            cur_y += facing[1]
            grid[cur_x][cur_y] = 'o'

        #Exit the loop for parsing
        elif input_dir.lower() == "x":
            parsing = False
        
        elif input_dir.lower() == 'p':
            print(grid)
            print(cur_x, cur_y)
        

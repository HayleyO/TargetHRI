#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi
from Grid import Grid, Directions

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

def move(grid, direction, cmd_vel, move_cmd, r):
    if direction == Directions.Up:
        if grid.check_move(direction):
            move_forward(1, move_cmd, cmd_vel, r)
        else: print('Invalid Movement')

    elif direction == Directions.Down:
        if grid.check_move(direction):
            move_forward(-1.1, move_cmd, cmd_vel, r)
        else: print('Invalid Movement')

    elif direction == Directions.Left:
        if grid.check_move(direction):
            rotate(-1.13, move_cmd, cmd_vel)
            move_forward(1, move_cmd, cmd_vel, r)
            rotate(1.127, move_cmd, cmd_vel)
        else: print('Invalid Movement')

    elif direction == Directions.right:
        if grid.check_move(direction):
            rotate(1.127, move_cmd, cmd_vel)
            move_forward(1, move_cmd, cmd_vel, r)
            rotate(-1.13, move_cmd, cmd_vel)
        else: print('Invalid Movement')


 
if __name__ == '__main__':
    rospy.init_node('Testing_Motion', anonymous=True)
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    move_cmd = Twist()
    r = rospy.Rate(2)
    grid = Grid()

    # maze_x, maze_y = 3, 3
    # cur_x, cur_y = 0, 0

    # facing = [0, 1]
    if raw_input('Parsing? y/n') in ['yes', 'y']:
        parsing = True
    while parsing:
        input_dir = raw_input("Give a direction: ")
        if input_dir.lower() in ["down", "d"]:
            move(grid, Directions.Down, cmd_vel, move_cmd)

        elif input_dir.lower() in ["up", "u"]:
            move(grid, Directions.Up, cmd_vel, move_cmd)

        elif input_dir.lower() in ["left", "l"]:
            move(grid, Directions.Left, cmd_vel, move_cmd)

        elif input_dir.lower() in ["right", "r"]:
            move(grid, Directions.Right, cmd_vel, move_cmd)

        #Exit the loop for parsing
        elif input_dir.lower() == "x":
            parsing = False
        
        elif input_dir.lower() == 'p':
            grid.print_grid()
        

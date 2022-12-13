#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi
from Grid import Grid, Directions

duration = 4
linear_speed = 0.4

class Turtlebot:
    '''
    Physical turtlebot that moves according to the directions
    '''
    def __init__(self, grid=Grid(dimension=3)):
        #Ros setup
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        self.move_cmd = Twist()
        self.r = rospy.Rate(2)
        self.grid = grid
    
    def move(self, direction):
        passable, _, _ = self.grid.check_move(row=None, col=None, direction=direction)
        if passable:
            if direction == Directions.Up:

                self.move_forward(-1.1)

                self.grid.move_robot(direction)

            elif direction == Directions.Down:
                self.move_forward(1.1)

                self.grid.move_robot(direction)

            elif direction == Directions.Left: 
                self.rotate(-1.13)
                self.move_forward(1.1)
                self.rotate(1.127)

                self.grid.move_robot(direction)

            elif direction == Directions.Right:                
                self.rotate(1.127)
                self.move_forward(1.1)
                self.rotate(-1.127)

                self.grid.move_robot(direction)
        else: print('Invalid Movement')

    def move_forward(self, x):
        # for _ in range(duration):
        self.move_cmd.linear.x = x*linear_speed
        self.move_cmd.angular.z = 0
        for _ in range(duration):
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

    def rotate(self, x):
        self.move_cmd.angular.z = x*pi*4/4
        self.move_cmd.linear.x = 0
        self.cmd_vel.publish(self.move_cmd)
        rospy.sleep(duration)

if __name__ == '__main__':

    rospy.init_node('Testing_Motion', anonymous=True)

    grid = Grid(dimension=3)
    tb = Turtlebot(grid)

    # maze_x, maze_y = 3, 3
    # cur_x, cur_y = 0, 0

    # facing = [0, 1]
    if raw_input('Parsing? y/n ') in ['yes', 'y']:
        parsing = True
    while parsing:
        input_dir = raw_input("Give a direction: ")
        if input_dir.lower() in ["down", "d"]:
            print("Got here")
            tb.move(Directions.Down)

        elif input_dir.lower() in ["up", "u"]:
            tb.move(Directions.Up)

        elif input_dir.lower() in ["left", "l"]:
            tb.move(Directions.Left)

        elif input_dir.lower() in ["right", "r"]:
            tb.move(Directions.Right)

        #Exit the loop for parsing
        elif input_dir.lower() == "x":
            parsing = False
        
        elif input_dir.lower() == 'p':
            grid.print_grid()


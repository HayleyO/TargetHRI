#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi
from Grid import Grid, Directions
from SaveAndLoadHelper import load, save
from Simple_RL import Q_Learning_RL_environment

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

        self.forward_distance_turns = 0.9
        self.forward_distance_down = 1.2
        self.forward_distance_up = 1.1
        self.rotate_angle = 1.13
    
    def move(self, direction, past_row=None, past_col=None):
        passable, _, _ = self.grid.check_move(row=past_row, col=past_col, direction=direction)
        if passable:
            if direction == Directions.Up:

                self.move_forward(-self.forward_distance_up)

            elif direction == Directions.Down:
                self.move_forward(self.forward_distance_down)

            elif direction == Directions.Left: 
                self.rotate(-self.rotate_angle)
                self.move_forward(self.forward_distance_turns)
                self.rotate(self.rotate_angle)

            elif direction == Directions.Right:                
                self.rotate(self.rotate_angle)
                self.move_forward(self.forward_distance_turns)
                self.rotate(-self.rotate_angle)

        else: print('Invalid Movement')

    def move_forward(self, x):
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

EPSILON = 0.5

if __name__ == '__main__':

    rospy.init_node('Testing_Motion', anonymous=True)

    grid = Grid(dimension=3)
    tb = Turtlebot(grid)

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
    else:

        rl = Q_Learning_RL_environment(grid=grid, epsilon=EPSILON, trust_alg=False, guidance=False)
        rl.load_q_table(name="Test_Err")
        done = False
        while not done:
            #To do integrate bumper
            done, action, past_state = rl.move_to_policy()
            past_row, past_col = rl.get_grid_position_from_state(past_state)
            print(action)
            tb.move(action, past_row, past_col)
            tb.grid.print_grid()

        rl.steps_per_episode.append(rl.steps)
        name = raw_input('What will this run be called? ')
        save(rl, path="SavedRuns/People", name="name")



'''#!/usr/bin/env python

import math
import rospy
from Grid import Grid, Directions


def rotate(speed, angle, clockwise=True):
    global cmd_pub

    adjustment_angle = 40
    angle = angle + adjustment_angle
    angular_speed = speed*2*math.pi/360
    relative_angle = angle*2*math.pi/360

    vel_msg = Twist()

    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        cmd_pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    cmd_pub.publish(vel_msg)

def moveRobot(grid, direction):
    global cmd_pub

    done = False
    passable, row, col = grid.check_move(None, None, direction)
    if passable:

        linear_speed = 0.2
        move_cmd = Twist()

        if direction == Directions.Down:
            duration = 2.5
            while duration > 0:

                move_cmd.linear.x = linear_speed
                move_cmd.angular.z = 0.0
                
                cmd_pub.publish(move_cmd)
                rate.sleep()

                duration = duration - 0.1

        elif direction == Directions.Right:
            rotate(90, 90, False)

            duration = 2.5
            while duration > 0:
                move_cmd.linear.x = linear_speed
                move_cmd.angular.z = 0.0
                cmd_pub.publish(move_cmd)
                rate.sleep()
                duration = duration - 0.1

            move_cmd.linear.x = 0.0
            cmd_pub.publish(move_cmd)
            rate.sleep()

            rotate(90, 90, True)

        elif Directions.Left:
            rotate(90, 90, True)

            duration = 2.5
            while duration > 0:
                move_cmd.linear.x = linear_speed
                move_cmd.angular.z = 0.0
                cmd_pub.publish(move_cmd)
                rate.sleep()
                duration = duration - 0.1

            move_cmd.linear.x = 0.0
            cmd_pub.publish(move_cmd)
            rate.sleep()

            rotate(90, 90, False)
        elif Direction.Up:
            rotate(90, 180, False)

            duration = 2.5
            while duration > 0:
                move_cmd.linear.x = linear_speed
                move_cmd.angular.z = 0.0
                cmd_pub.publish(move_cmd)
                rate.sleep()
                duration = duration - 0.1

            move_cmd.linear.x = 0.0
            cmd_pub.publish(move_cmd)
            rate.sleep()

            rotate(90, 180, True)

        done = grid.move_robot(direction)

    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0

    cmd_pub.publish(move_cmd)
    rate.sleep()
    return done


if __name__ == '__main__':
    rospy.init_node('Move_In_Grid', anonymous=True)
    cmd_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rate = rospy.Rate(10)

    grid = Grid(dimension=3)

    parsing = True 
    while parsing:
        direction = None
        input_dir = raw_input("Give a direction: ")
        if input_dir.lower() in ["down", "d", "up", "u", "right", "r", "left", "l"]:
            if input_dir.lower() in ["down", "d"]:
                direction = Directions.Down
                done = moveRobot(grid, direction)
            elif input_dir.lower() in ["up", "u"]:
                direction = Directions.Up
                done = moveRobot(grid, direction)
            elif input_dir.lower() in ["right", "r"]:
                direction = Directions.Right
                done = moveRobot(grid, direction)
            elif input_dir.lower() in ["left", "l"]:
                direction = Directions.Left
                done = moveRobot(grid, direction)
        elif input_dir.lower() == 'x':
            parsing = False
        else:
            print("Please try again! I didn't quite understand that. Try: left, right, up, down or just the first letter of them.")'''
        


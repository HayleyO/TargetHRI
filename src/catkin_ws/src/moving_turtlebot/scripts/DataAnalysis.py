from SaveAndLoadHelper import load, load_rewards, load_q_table

import math
import numpy as np

def print_rewards(rewards):
    print(rewards)

def print_steps(steps):
    print(steps)

def print_start_end_steps(steps):
    print("Starting Amount of Steps: " + str(steps[0]))
    print("Ending Amount of Steps: " + str(steps[-1]))


def ratio(value1, value2, print_ratio=True):
    if value2 != 0:
        ratio = value1/value2
    else:
        ratio = -math.inf
        
    if print_ratio:
        print(str(value1) +"/"+ str(value2) + ": " + str(ratio))
    return ratio

def euclidean_dist(value1, value2, print_dist=True):
    dist = math.dist([value1], [value2])
    if print_dist:
        print("Distance: " + str(dist))
    return dist

def multiplication(value1, value2, print_mult=True):
    result = value1*value2
    if print_mult:
        print("Multiplication: " + str(result))
    return result

def average(value_list, print_average=True):
    sum = 0
    for value in value_list:
        sum = sum + value
    average = sum/len(value_list)
    if print_average:
        print("Average: " + str(average))
    return average

def normalize(q_table):
    return (q_table-np.min(q_table))/(np.max(q_table)-np.min(q_table))

if __name__ == "__main__":
   print("Eh")

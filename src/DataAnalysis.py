from SaveAndLoadHelper import load, load_rewards, load_q_table
from statistics import mean
import pandas
import math
import numpy as np

def display_q_table(q_table):
    col_labels = ['Guidance', 'Up', 'Down', 'Left', 'Right']
    df = pandas.DataFrame(q_table, columns=col_labels)
    print(df)

def print_rewards(rewards):
    print(rewards)

def print_steps(steps):
    print(steps)

def print_start_end_steps(steps):
    print("Starting Amount of Steps: " + str(steps[0]))
    print("Ending Amount of Steps: " + str(steps[-1]))

def print_average_steps(steps):
    print("Average steps: " + str(mean(steps)))

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
    q_table_truth, rewards_truth, steps_truth, _, _ = load(name="Test_Truth")
    q_table_lie, rewards_lie, steps_lie, _, _ = load(name="Test_Lie")
    q_table_err, rewards_err, steps_err, _, _ = load(name="Test_Err")
    #q_table_test_no_guide, rewards_test_no_guide, steps_test_no_guide, _, _ = load(name="Test_No_Guide")
    #q_table_test_err_human_w_alg_truth, rewards_test_err_human_w_alg_truth, steps_test_err_human_w_alg_truth, _, _ = load(name="Test_Err_Human_w_alg_true")
    #q_table_test_err_human_w_alg_lie, rewards_test_err_human_w_alg_lie, steps_test_err_human_w_alg_lie, _, _ = load(name="Test_Err_Human_w_alg_lie")

    display_q_table(q_table_truth)
    display_q_table(q_table_lie)
    display_q_table(q_table_err)
    #display_q_table(q_table_test_no_guide)
    #display_q_table(q_table_test_err_human_w_alg_truth)
    #display_q_table(q_table_test_err_human_w_alg_lie)

    normalized_q_true = normalize(q_table_truth)
    display_q_table(normalized_q_true)
    normalized_q_lie = normalize(q_table_lie)
    display_q_table(normalized_q_lie)
    normalized_q_err = normalize(q_table_err)
    display_q_table(normalized_q_err)
    #print_rewards(q_table_truth)
    #print_rewards(q_table_lie)
    #print_rewards(q_table_err)

    print_start_end_steps(steps_truth)
    print_start_end_steps(steps_lie)
    print_start_end_steps(steps_err)
    #print_start_end_steps(steps_test_no_guide)
    #print_start_end_steps(steps_test_err_human_w_alg_truth)
    #print_start_end_steps(steps_test_err_human_w_alg_lie)


    #print_average_steps(steps_truth)
    #print_average_steps(steps_lie)
    #print_average_steps(steps_err)
    #print_average_steps(steps_test_no_guide)

from SaveAndLoadHelper import load, load_rewards, load_q_table
from statistics import mean
import pandas
import math

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

if __name__ == "__main__":
    q_table_truth, rewards_truth, steps_truth = load(name="Test_Truth")
    q_table_lie, rewards_lie, steps_lie = load(name="Test_Lie")
    q_table_err, rewards_err, steps_err = load(name="Test_Err")
    q_table_test_no_guide, rewards_test_no_guide, steps_test_no_guide = load(name="Test_No_Guide")

    display_q_table(q_table_truth)
    display_q_table(q_table_lie)
    display_q_table(q_table_err)
    display_q_table(q_table_test_no_guide)


    #print_rewards(q_table_truth)
    #print_rewards(q_table_lie)
    #print_rewards(q_table_err)

    print_start_end_steps(steps_truth)
    print_start_end_steps(steps_lie)
    print_start_end_steps(steps_err)
    print_start_end_steps(rewards_test_no_guide)


    print_average_steps(steps_truth)
    print_average_steps(steps_lie)
    print_average_steps(steps_err)
    print_average_steps(steps_test_no_guide)
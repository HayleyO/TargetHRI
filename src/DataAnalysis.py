from SaveAndLoadHelper import load, load_rewards, load_q_table
import pandas

def display_q_table(q_table):
    col_labels = ['Guidance', 'Up', 'Down', 'Left', 'Right']
    df = pandas.DataFrame(q_table, columns=col_labels)
    print(df)

def print_average_rewards(rewards):
    pass

if __name__ == "__main__":
    q_table_truth, rewards_truth = load(name="Truth_Test")
    q_table_lie, rewards_lie = load(name="Lie_Test")

    display_q_table(q_table_truth)
    display_q_table(q_table_lie)
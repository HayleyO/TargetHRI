from Simple_RL import Q_Learning_RL_environment
from Oracle import Oracle
from Grid import Grid
import numpy as np
import pickle
import os

def save(rl, path="SavedRuns", name="test"):
    q_table = rl.q_table # The q_table from the reinforcement learning
    rewards = rl.rewards_per_episode
    save_q_table(q_table, path, name)
    save_rewards(rewards, path, name)

def save_q_table(q_table, path="SavedRuns", name="test"):
    assert(isinstance(q_table, np.ndarray)) # Ensure that it is a numpy array
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(source_folder, path) # Create full path
    if not(os.path.exists(full_path)):
        os.mkdir(full_path)
    path_with_name =  os.path.join(full_path, name +"_q_table.npy")
    np.save(path_with_name, q_table) # Save final q_table

def save_rewards(rewards, path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(source_folder, path) # Create full path
    if not(os.path.exists(full_path)):
        os.mkdir(full_path)
    path_with_name =  os.path.join(full_path, name +"_rewards")
    with open(path_with_name, "wb") as fp:   #Pickling
        pickle.dump(rewards, fp)

def load_q_table(path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path_with_name = os.path.join(source_folder, path, name +"_q_table.npy") # Create full path
    return np.load(full_path_with_name) 

def load_rewards(path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path_with_name = os.path.join(source_folder, path, name +"_rewards") # Create full path
    with open(full_path_with_name, "rb") as fp:   # Unpickling
        rewards = pickle.load(fp)
    return rewards

def load(path="SavedRuns", name="test"):
    q_table = load_q_table(path, name)
    rewards = load_rewards(path, name)
    return q_table, rewards

if __name__ == "__main__":
    rl_truthful = Q_Learning_RL_environment(grid = Grid(5), oracle=Oracle('truthful'))
    _ = rl_truthful.run_episodes()
    print("Truthful Oracle Done")
    save(rl_truthful, name="Truth_Test")

    rl_lie = Q_Learning_RL_environment(grid = Grid(5), oracle=Oracle('lying'))
    _ = rl_lie.run_episodes()
    print("Lying Oracle Done")
    save(rl_lie, name="Lie_Test")

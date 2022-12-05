import numpy as np
import pickle
import os

def save(rl, path="SavedRuns", name="test"):
    q_table = rl.q_table # The q_table from the reinforcement learning
    rewards = rl.rewards_per_episode
    steps = rl.steps_per_episode
    lie_count = rl.lie_count
    guide_count = rl.guidance_count
    save_q_table(q_table, path, name)
    save_rewards(rewards, path, name)
    save_steps(steps, path, name)
    save_lie_count(lie_count, path, name)
    save_guide_count(guide_count, path, name)

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

def save_steps(steps, path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(source_folder, path) # Create full path
    if not(os.path.exists(full_path)):
        os.mkdir(full_path)
    path_with_name =  os.path.join(full_path, name +"_steps")
    with open(path_with_name, "wb") as fp:   #Pickling
        pickle.dump(steps, fp)

def save_lie_count(lie_count, path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(source_folder, path) # Create full path
    if not(os.path.exists(full_path)):
        os.mkdir(full_path)
    path_with_name =  os.path.join(full_path, name +"_lie_count")
    with open(path_with_name, "wb") as fp:   #Pickling
        pickle.dump(lie_count, fp)

def save_guide_count(guide_count, path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(source_folder, path) # Create full path
    if not(os.path.exists(full_path)):
        os.mkdir(full_path)
    path_with_name =  os.path.join(full_path, name +"_guide_count")
    with open(path_with_name, "wb") as fp:   #Pickling
        pickle.dump(guide_count, fp)

def save_epsilon(epsilon, path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(source_folder, path) # Create full path
    if not(os.path.exists(full_path)):
        os.mkdir(full_path)
    path_with_name =  os.path.join(full_path, name +"_epsilon")
    with open(path_with_name, "wb") as fp:   #Pickling
        pickle.dump(epsilon, fp)

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

def load_steps(path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path_with_name = os.path.join(source_folder, path, name +"_steps") # Create full path
    with open(full_path_with_name, "rb") as fp:   # Unpickling
        steps = pickle.load(fp)
    return steps

def load_lie_count(path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path_with_name = os.path.join(source_folder, path, name +"_lie_count") # Create full path
    with open(full_path_with_name, "rb") as fp:   # Unpickling
        lie_count = pickle.load(fp)
    return lie_count

def load_guide_count(path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path_with_name = os.path.join(source_folder, path, name +"_guide_count") # Create full path
    with open(full_path_with_name, "rb") as fp:   # Unpickling
        guide_count = pickle.load(fp)
    return guide_count

def load_epsilon(path="SavedRuns", name="test"):
    source_folder = os.path.dirname(os.path.abspath(__file__))
    full_path_with_name = os.path.join(source_folder, path, name +"_epsilon") # Create full path
    with open(full_path_with_name, "rb") as fp:   # Unpickling
        epsilon = pickle.load(fp)
    return epsilon

def load(path="SavedRuns", name="test"):
    q_table = load_q_table(path, name)
    rewards = load_rewards(path, name)
    steps = load_steps(path, name)
    lie_count = load_lie_count(path, name)
    guide_count = load_guide_count(path, name)
    return q_table, rewards, steps, lie_count, guide_count


import random
import numpy as np
from enum import Enum
from Grid import Grid, Directions
from Oracle import Oracle
from SaveAndLoadHelper import save, load_q_table
from DataAnalysis import ratio, euclidean_dist

class Q_Learning_RL_environment():

    def __init__(self, episodes=10000, grid = Grid(), oracle=None):
        # States: dimension x dimension grid
        self.grid = grid
        self.oracle = oracle
        self.current_state = self.get_state_from_grid_position()
        # Actions: [Ask_For_Guidance, Go_Left, Go_Right, Go_Up, Go_Down]
        self.actions = {Actions.Ask_For_Guidance: "n/a", Actions.Up: Directions.Up, Actions.Down: Directions.Down, Actions.Left: Directions.Left, Actions.Right: Directions.Right}
        # Rewards: No rewards for taking action? Maybe -1, -100 if 100 turns pass and not done. +100 if at terminating state
        self.termination_correct_reward = 100
        self.termination_incorrect_reward = -100
        self.non_termination_reward = -1
        
        self.q_table = np.zeros(((self.grid.width*self.grid.height), len(self.actions)))
        self.rewards_per_episode = []
        self.steps_per_episode = []

        # Hyperparamaters
        self.n_episodes = episodes
        self.max_turn = 100
        self.epsilon = 1 # Exploration (for epsilon exploration and decay)
        self.lr = 0.1 # Learning rate (for q_table calculations)
        self.gamma = 0.99 # Discount factor (for q_table calculations)
        self.min_explore_prob = 0.01 # Sets minimum for epsilon decay (for epsilon exploration and decay)
        self.exploration_decreasing_decay = 0.001 # How much to decay (for epsilon exploration and decay)

    def action_index(self, action):
        if action == Actions.Ask_For_Guidance:
            return 0
        elif action == Actions.Up:
            return 1
        elif action == Actions.Down:
            return 2                
        elif action == Actions.Left:
            return 3
        elif action == Actions.Right:
            return 4
        else:
            return -1

    def action_from_index(self, action_index):
        if action_index == 0:
            return Actions.Ask_For_Guidance
        elif action_index == 1:
            return Actions.Up
        elif action_index == 2:
            return Actions.Down                
        elif action_index == 3:
            return Actions.Left
        elif action_index == 4:
            return Actions.Right
        else:
            return -1

    def action_from_direction(self, action_direction):
        if action_direction == Directions.Up:
            return Actions.Up
        if action_direction == Directions.Down:
            return Actions.Down
        if action_direction == Directions.Left:
            return Actions.Left 
        if action_direction == Directions.Right:
            return Actions.Right

    def get_state_from_grid_position(self):
        row, col = self.grid.get_robot_location()
        self.current_state = ((row*self.grid.height) + col)
        return self.current_state

    # Q-Learning MDP
    def run_episodes(self):
        rewards_per_episode = []
        for e in range(self.n_episodes):
            self.grid.refresh_grid()
            current_step_state = self.get_state_from_grid_position()
            done = False    

            step_i = 0
            total_episode_reward = 0
            for step in range(self.max_turn):

                # This is called greedy epsilon, it takes random actions to "explore"
                if np.random.uniform(0,1) < self.epsilon:
                    action, _ = random.choice(list(self.actions.items()))
                else:
                    action_index = np.argmax(self.q_table[current_step_state,:])
                    action = self.action_from_index(action_index)
                
                next_state, reward, done = self.take_action(action)
                action_index = self.action_index(action) # Get table index of actions
                # Update q table
                # Potential "changed/improved" algorithm problem: action will be "guidance" but it won't be specific from guidance -> guided action
                self.q_table[current_step_state, action_index] = (1-self.lr) * self.q_table[current_step_state, action_index] + self.lr*(reward + self.gamma*max(self.q_table[next_state,:]))
                
                if step == self.max_turn -1:
                    total_episode_reward = total_episode_reward + self.termination_incorrect_reward
                else:
                    total_episode_reward = total_episode_reward + reward

                if done:
                    step_i = step
                    break

                current_step_state = next_state # While technically the robot is in the next_state at this point, this updates it logically
                self.grid.print_grid()
                step_i = step
            self.steps_per_episode.append(step_i)
            self.epsilon = max(self.min_explore_prob, np.exp(-self.exploration_decreasing_decay*e))
            rewards_per_episode.append(total_episode_reward)
        self.rewards_per_episode = rewards_per_episode
        return rewards_per_episode

        
    def take_action(self, action):
        if action == Actions.Ask_For_Guidance:
            if self.oracle == None:
                self.grid.print_grid()
                parsing = True
                while parsing:
                    user_guidance = input("What move should the robot make (Left, Right, Up, Down)?: ")
                    if user_guidance.lower() in ["left", "l", "right", "r", "up", "u", "down", "d"]:
                        if user_guidance.lower() in["up", "u"]:
                            action = Actions.Up
                        elif user_guidance.lower() in["down", "d"]:
                            action = Actions.Down
                        elif user_guidance.lower() in["left", "l"]:
                            action = Actions.Left
                        elif user_guidance.lower() in ['right', 'r']:
                            action = Actions.Right
                        parsing = False
                    else:
                        print("I don't recognize that, sorry, try again: ")

                curr_state = self.get_state_from_grid_position()
                cur_guide_q = self.q_table[curr_state][self.action_index(Actions.Ask_For_Guidance)]
                cur_dir_q = self.q_table[curr_state][self.action_index(action)]
                print("Current Guide Q: " + str(cur_guide_q))
                print("Curent Direction Q: " + str(cur_dir_q))

                current_decision = (cur_dir_q - cur_guide_q)
                print("Curent Decision Q: " + str(current_decision))

                q_table_truth = load_q_table(name="Test_Truth")
                true_guide_q = q_table_truth[curr_state][self.action_index(Actions.Ask_For_Guidance)]
                true_dir_q = q_table_truth[curr_state][self.action_index(action)]
                print("True Guide Q: " + str(true_guide_q))
                print("True Direction Q: " + str(true_dir_q))

                truth_decision = (true_dir_q - true_guide_q)
                print("Truth Decision Q: " + str(truth_decision))

                q_table_lie = load_q_table(name="Test_Lie")
                lie_guide_q = q_table_lie[curr_state][self.action_index(Actions.Ask_For_Guidance)]
                lie_dir_q = q_table_lie[curr_state][self.action_index(action)]
                print("Lie Guide Q: " + str(lie_guide_q))
                print("Lie Direction Q: " + str(lie_dir_q))

                lie_decision = (lie_dir_q - lie_guide_q)
                print("Lie Decision Q: " + str(lie_decision))

                print()
                print()
                print()

                print("Truth Similarity")
                ratio(current_decision, truth_decision)
                ratio(truth_decision, current_decision)
                true_dist = euclidean_dist(truth_decision, current_decision)

                print("Lie Similarity")
                ratio(current_decision, lie_decision)
                ratio(lie_decision, current_decision)
                lie_dist = euclidean_dist(lie_decision, current_decision)

                if lie_dist <= true_dist:
                    m = np.zeros(len(self.actions), dtype=bool)
                    m[0] = True
                    a = np.ma.array(self.q_table[curr_state,:], mask=m)
                    action_index = np.argmax(a)
                    action = self.action_from_index(action_index)

            else: action = self.action_from_direction(self.oracle.give_advice(self.grid))
        termination_state = self.grid.move_robot(self.actions[action])
        self.current_state = self.get_state_from_grid_position()
        if termination_state:
            return self.current_state, self.termination_correct_reward, termination_state
        else:
            return self.current_state, self.non_termination_reward, termination_state


class Actions(Enum):
    Ask_For_Guidance = 0,
    Up = 1,
    Down = 2,
    Left = 3,
    Right = 4   
    

if __name__ == "__main__":
    
    rl_truth = Q_Learning_RL_environment(oracle=Oracle('truthful'))
    _ = rl_truth.run_episodes()
    save(rl_truth, name="Test_Truth")

    rl_lie = Q_Learning_RL_environment(oracle=Oracle('lying'))
    _ = rl_lie.run_episodes()
    save(rl_lie, name="Test_Lie")

    rl_err = Q_Learning_RL_environment(oracle=Oracle('erroneous'))
    _ = rl_err.run_episodes()
    save(rl_err, name="Test_Err")

    #rl = Q_Learning_RL_environment(episodes=1)
    #rewards_per_episode = rl.run_episodes()

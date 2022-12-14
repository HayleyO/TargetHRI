from Simple_RL import Q_Learning_RL_environment
from SaveAndLoadHelper import save, load
from DataAnalysis import average
from Grid import Grid

EPSILON = 0.5

if __name__ == "__main__":
    
    name = input("What is your name? ")

    print("Give true guidance")
    grid = Grid()
    rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON)
    rl.load_q_table(name="Test_Err")
    rewards_per_episode= rl.run_episodes(print_grid=False, print_data=False, train=False)
    save(rl, path="SavedRuns/Person", name="True_Guidance_w_Alg_"+ str(name))

    print("Give lying guidance")
    grid = Grid()
    rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON)
    rl.load_q_table(name="Test_Err")
    rewards_per_episode= rl.run_episodes(print_grid=False, print_data=False, train=False)
    save(rl, path="SavedRuns/Person", name="Lie_Guidance_w_Alg_"+ str(name))

    print("-" * 45)
    print("Give true guidance")
    grid = Grid()
    rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON, trust_alg=False)
    rl.load_q_table(name="Test_Err")
    rewards_per_episode= rl.run_episodes(print_grid=False, print_data=False, train=False)
    save(rl, path="SavedRuns/Person", name="True_Guidance_without_Alg_"+ str(name))

    print("-" * 45)
    print("Give lying guidance")
    grid = Grid()
    rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON, trust_alg=False)
    rl.load_q_table(name="Test_Err")
    rewards_per_episode= rl.run_episodes(print_grid=False, print_data=False, train=False)
    save(rl, path="SavedRuns/Person", name="Lie_Guidance_without_Alg_"+ str(name))

    print("-" * 45)
    print("Doing a run with no guidance")
    grid = Grid()
    rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON, trust_alg=False, guidance=False)
    rl.load_q_table(name="Test_Err")
    rewards_per_episode= rl.run_episodes(print_grid=False, print_data=False, train=False)
    save(rl, path="SavedRuns/Person", name="No_Guidance_"+ str(name))
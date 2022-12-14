from Simple_RL import Q_Learning_RL_environment
from SaveAndLoadHelper import save, load
from DataAnalysis import average
from Grid import Grid

if __name__ == "__main__":

    RUNS = 10
    TRAIN = False
    PRINT_DATA = False
    PRINT_GRID = False
    EPSILON = 0.5

    if TRAIN:
        # Algorithm w/ true human feedback
        print("Give true guidance")
        for run in range(RUNS):
            print("Run " + str(run) + ":")
            grid = Grid()
            #grid.set_element(1,3,grid.blocked_token)
            #grid.set_element(2,2,grid.blocked_token)
            #grid.set_element(3,1,grid.blocked_token)
            rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON)
            rl.load_q_table(name="Test_Err")
            #rl.load_epsilon(name="Test_Err")
            rewards_per_episode= rl.run_episodes(print_grid=PRINT_GRID, print_data=PRINT_DATA, train=False)
            save(rl, path="SavedRuns/Analysis", name="Test_Err_Human_w_alg_True_"+str(run))

        # Algorithm w/ false human feedback
        print("Give false guidance")
        for run in range(RUNS):
            print("Run " + str(run) + ":")
            grid = Grid()
            #grid.set_element(1,3,grid.blocked_token)
            #grid.set_element(2,2,grid.blocked_token)
            #grid.set_element(3,1,grid.blocked_token)
            rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON)
            rl.load_q_table(name="Test_Err")
            #rl.load_epsilon(name="Test_Err")
            rewards_per_episode = rl.run_episodes(print_grid=PRINT_GRID, print_data=PRINT_DATA, train=False)
            save(rl, path="SavedRuns/Analysis", name="Test_Err_Human_w_alg_Lie_"+str(run))

        # No algorithm w/ true human feedback
        print("Give true guidance")
        for run in range(RUNS):
            print("Run " + str(run) + ":")
            grid = Grid()
            #grid.set_element(1,3,grid.blocked_token)
            #grid.set_element(2,2,grid.blocked_token)
            #grid.set_element(3,1,grid.blocked_token)
            rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON, trust_alg=False)
            rl.load_q_table(name="Test_Err")
            #rl.load_epsilon(name="Test_Err")
            rewards_per_episode = rl.run_episodes(print_grid=PRINT_GRID, print_data=PRINT_DATA, train=False)
            save(rl, path="SavedRuns/Analysis", name="Test_Err_Human_w_o_alg_True_"+str(run))

        # No algorithm w/ false human feedback
        print("Give false guidance")
        for run in range(RUNS):
            print("Run " + str(run) + ":")
            grid = Grid()
            #grid.set_element(1,3,grid.blocked_token)
            #grid.set_element(2,2,grid.blocked_token)
            #grid.set_element(3,1,grid.blocked_token)
            rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON, trust_alg=False)
            rl.load_q_table(name="Test_Err")
            #rl.load_epsilon(name="Test_Err")
            rewards_per_episode = rl.run_episodes(print_grid=PRINT_GRID, print_data=PRINT_DATA, train=False)
            save(rl, path="SavedRuns/Analysis", name="Test_Err_Human_w_o_alg_Lie_"+str(run))

        # No algorithm w/ no human feedback
        for run in range(RUNS):
            #print("Run " + str(run) + ":")
            grid = Grid()
            #grid.set_element(1,3,grid.blocked_token)
            #grid.set_element(2,2,grid.blocked_token)
            #grid.set_element(3,1,grid.blocked_token)
            rl = Q_Learning_RL_environment(grid=grid, episodes=1, epsilon=EPSILON, trust_alg=False, guidance=False)
            rl.load_q_table(name="Test_Err")
            #rl.load_epsilon(name="Test_Err")
            rewards_per_episode = rl.run_episodes(print_grid=PRINT_GRID, print_data=PRINT_DATA, train=False)
            save(rl, path="SavedRuns/Analysis", name="Test_Err_no_Human"+str(run))

    truth_steps = []
    truth_accuracy = []        
    for run in range(RUNS):
        q_table_truth, rewards_truth, steps_truth, lie_count_true, guide_count_true = load(path="SavedRuns/Analysis", name="Test_Err_Human_w_alg_True_"+str(run))
        truth_steps.append(steps_truth[0])
        if guide_count_true != 0:
            truth_accuracy.append((guide_count_true-lie_count_true)/guide_count_true)
    
    lie_steps = []
    lie_accuracy = []
    for run in range(RUNS):
        q_table_lie, rewards_lie, steps_lie, lie_count_lie, guide_count_lie = load(path="SavedRuns/Analysis", name="Test_Err_Human_w_alg_Lie_"+str(run))
        lie_steps.append(steps_lie[0])
        if guide_count_lie != 0:
            lie_accuracy.append(lie_count_lie/guide_count_lie)

    truth_no_alg_steps = []
    for run in range(RUNS):
        q_table_truth, rewards_truth, steps_truth_no_alg, lie_count_true_no_alg, guide_count_true_no_alg = load(path="SavedRuns/Analysis", name="Test_Err_Human_w_o_alg_True_"+str(run))
        truth_no_alg_steps.append(steps_truth_no_alg[0])
    
    lie_no_alg_steps = []
    for run in range(RUNS):
        q_table_lie, rewards_lie, steps_lie_no_alg, lie_count_lie_no_alg, guide_count_lie_no_alg = load(path="SavedRuns/Analysis", name="Test_Err_Human_w_o_alg_Lie_"+str(run))
        lie_no_alg_steps.append(steps_lie_no_alg[0])

    no_guide_steps = []
    for run in range(RUNS):
        q_table_lie, rewards_lie, steps_no_guide, _, _ = load(path="SavedRuns/Analysis", name="Test_Err_no_Human"+str(run))
        no_guide_steps.append(steps_no_guide[0])
    
    print("Truth:")
    print("Steps: " + str(average(truth_steps, False)))
    print("Accuracy: " + str(average(truth_accuracy, False)))

    print("Lie:")
    print("Steps: " + str(average(truth_no_alg_steps, False)))
    print("Accuracy: " + str(average(lie_accuracy, False)))

    print("Truth no Algorithm:")
    print("Steps: " + str(average(lie_steps, False)))

    print("Lie no Algorithm:")
    print("Steps: " + str(average(lie_no_alg_steps, False)))

    print("No Guide:")
    print("Steps: " + str(average(no_guide_steps, False)))# Compare steps 
# Compare percent accuracy
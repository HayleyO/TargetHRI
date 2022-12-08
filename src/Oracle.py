from Grid import Grid, Directions
from enum import Enum 
import math
import random

class Oracle:
    '''
    Untrustworthy oracle that generates advice wrt its mode and A* search result.
    '''

    def __init__(self, mode='truthful', error_rate=0.5):
        assert mode in ['truthful', 'lying', 'erroneous']
        assert error_rate>=0 and error_rate <= 1
        self.mode = mode
        self.error_rate = error_rate
    
    def change_mode(self, mode):
        assert mode in ['truthful', 'lying', 'erroneous']
        self.mode = mode 

    def manhattan_distance(self, loc1, loc2):
        '''
        Compute the Mahattan disctance between two locations
        '''
        return abs(loc1[0]-loc2[0])+abs(loc1[1]-loc2[1])
    
    def euclidean_distance(self, loc1, loc2):
        '''
        Compute the Euclidean distance between two locations
        '''
        return math.sqrt((loc1[0]-loc2[0])**2 + (loc2[1]-loc1[1])**2)
    
    def get_candidate_values(self, grid):
        '''
        Compute the A* value of possible moving directions
        '''
        assert isinstance(grid, Grid)
        robot_loc = grid.get_robot_location()
        target_loc = grid.get_target_location()
        candidate_values = {}
        f = self.manhattan_distance(robot_loc, (0, 0))+1
        if robot_loc[0]>0:
            candidate_values[Directions.Up] = self.euclidean_distance((robot_loc[0]-1, robot_loc[1]), target_loc)+f
        if robot_loc[0]<grid.height-1:
            candidate_values[Directions.Down] = self.euclidean_distance((robot_loc[0]+1, robot_loc[1]), target_loc)+f
        if robot_loc[1]>0:
            candidate_values[Directions.Left] = self.euclidean_distance((robot_loc[0], robot_loc[1]-1), target_loc)+f
        if robot_loc[1]<grid.width-1:
            candidate_values[Directions.Right] = self.euclidean_distance((robot_loc[0], robot_loc[1]+1), target_loc)+f
        return candidate_values

    def give_advice(self, grid):
        '''
        Returns Directions according to oracle type
        '''
        candidate_values = self.get_candidate_values(grid)
        #print(candidate_values)
        MIN, MAX = 99999999, -1
        best, worst = None, None

        for i in reversed(Directions):
            if i not in candidate_values: continue
            if candidate_values[i] < MIN:
                MIN = candidate_values[i]
                best = i
            if candidate_values[i] > MAX:
                MAX = candidate_values[i]
                worst = i

        if self.mode == 'truthful': return best
        if self.mode == 'lying': return worst
        if self.mode == 'erroneous': return worst if random.random() < self.error_rate else best

        return None 

if __name__ == '__main__':
    grid = Grid()
    oracle = Oracle('lying')
    grid.print_grid()
    adv = oracle.give_advice(grid)
    count = 0
    while not grid.move_robot(adv):
        grid.print_grid()
        if count>50: break
        count += 1
        adv = oracle.give_advice(grid)
        print(adv)
    


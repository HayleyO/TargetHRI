from enum import Enum

class Grid:

    def __init__(self, dimension=5):
        self.height = dimension # Rows
        self.width = dimension # Columns
        self.bot_token = "o"
        self.empty_token = " "
        self.target_token = "*"
        self.blocked_token = "#"
        self.grid = self.create_grid()
    
    def create_grid(self):
        grid = [[self.empty_token for _ in range(self.width)] for _ in range(self.height)]
        grid[0][0] = self.bot_token
        grid[self.height-1][self.width-1] = self.target_token
        return grid

    def refresh_grid(self):
        self.grid = self.create_grid()

    def get_element(self, row, col):
        return self.grid[row][col]

    def set_element(self, row, col, val):
        self.grid[row][col] = val

    def get_robot_location(self):
        for row in range(len(self.grid)):
            for col in range(len(self.grid[row])):
                if self.get_element(row, col) == self.bot_token:
                    return (row, col)
        return (-1, -1)
    
    def get_target_location(self):
        for row in range(len(self.grid)):
            for col in range(len(self.grid[row])):
                if self.get_element(row, col) == self.target_token:
                    return (row, col)
        return (-1, -1)

    def move_robot(self, direction):
        '''
        Moves robot in a direction, returns true if it's terminating
        '''
        row, col = self.get_robot_location()
        if direction == Directions.Up:     
            if row > 0:
                next_space = self.get_element(row-1, col)
                if next_space != self.blocked_token:
                    self.set_element(row, col, self.empty_token)
                    row = row - 1
        elif direction == Directions.Down:
            if row < self.height-1:
                next_space = self.get_element(row+1, col)
                if next_space != self.blocked_token:
                    self.set_element(row, col, self.empty_token)
                    row = row + 1
        elif direction == Directions.Left:
            if col > 0:
                next_space = self.get_element(row, col-1)
                if next_space != self.blocked_token:
                    self.set_element(row, col, self.empty_token)
                    col = col - 1
        elif direction == Directions.Right:
            if col < self.width-1:
                next_space = self.get_element(row, col+1)
                if next_space != self.blocked_token:
                    self.set_element(row, col, self.empty_token)
                    col = col + 1
        row_target, col_target = self.get_target_location()
        self.set_element(row, col, self.bot_token)
        return (row == row_target and col == col_target)

    def check_move(self, row, col, direction):
        if row == None or col == None:
            row, col = self.get_robot_location()
        row_new = row
        col_new = col
        if direction == Directions.Up:     
            if row > 0:
                next_space = self.get_element(row-1, col)
                if next_space != self.blocked_token:
                    row_new = row - 1
        elif direction == Directions.Down:
            if row < self.height-1:
                next_space = self.get_element(row+1, col)
                if next_space != self.blocked_token:
                    row_new = row + 1
        elif direction == Directions.Left:
            if col > 0:
                next_space = self.get_element(row, col-1)
                if next_space != self.blocked_token:
                    col_new = col - 1
        elif direction == Directions.Right:
            if col < self.width-1:
                next_space = self.get_element(row, col+1)
                if next_space != self.blocked_token:
                    col_new = col + 1
        return (row != row_new or col != col_new), row_new, col_new

    def print_grid(self):
        for row in self.grid:
            row_print = ""
            for col in row:
                row_print = row_print + str(col) + " | "
            print(row_print)
            print("-" * 21)
        print(" ")
        print(" ")

class Directions(Enum):
    Up = 1
    Down = 2
    Left = 3
    Right = 4

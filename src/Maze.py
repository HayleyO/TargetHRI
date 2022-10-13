# Maze generator -- Randomized Prim Algorithm
# Code and Algorithm taken from: https://github.com/OrWestSide/python-scripts/blob/master/maze.py

## Imports
import random
from colorama import init, Fore


class Maze:

	def __init__(self, height=5, width=5):
		# Set up maze tokens
		self.wall_token = 'w'
		self.cell_token = 'c'
		self.unvisited_token = 'u'
		# Set height and width of maze
		self.height = height
		self.width = width
		self.maze = self.generate_blank_maze() # Fill empty array with "unvisited" tokens

	def generate_blank_maze(self):
		# Denote all cells as unvisited
		maze = []
		for _ in range(0, self.height):
			row = []
			for _ in range(0, self.width):
				row.append(self.unvisited_token)
			maze.append(row)
		return maze

	def set_starting_point(self):
		# Randomize starting point and set it a cell
		starting_height = int(random.randint(1, self.height-1))
		starting_width = int(random.randint(1, self.width-1))

		# Mark it as cell and add surrounding walls to the list
		self.maze[starting_height][starting_width] = self.cell_token
		walls = []
		walls.append([starting_height - 1, starting_width])
		walls.append([starting_height, starting_width - 1])
		walls.append([starting_height, starting_width + 1])
		walls.append([starting_height + 1, starting_width])

		# Denote walls in maze
		self.maze[starting_height-1][starting_width] = self.wall_token
		self.maze[starting_height][starting_width - 1] = self.wall_token
		self.maze[starting_height][starting_width + 1] = self.wall_token
		self.maze[starting_height + 1][starting_width] = self.wall_token
		return walls

	def get_surrounding_cells(self, rand_wall):
		s_cells = 0
		if (self.maze[rand_wall[0]-1][rand_wall[1]] == self.cell_token):
			s_cells += 1
		if (self.maze[rand_wall[0]+1][rand_wall[1]] == self.cell_token):
			s_cells += 1
		if (self.maze[rand_wall[0]][rand_wall[1]-1] == self.cell_token):
			s_cells +=1
		if (self.maze[rand_wall[0]][rand_wall[1]+1] == self.cell_token):
			s_cells += 1
		return s_cells

	def generate_maze(self):
		walls = self.set_starting_point()
		while (walls):
			# Pick a random wall
			rand_wall = walls[int(random.random()*len(walls))-1]

			# Check if it is a left wall
			if (rand_wall[1] != 0):
				if (self.maze[rand_wall[0]][rand_wall[1]-1] == self.unvisited_token and self.maze[rand_wall[0]][rand_wall[1]+1] == self.cell_token):
					# Find the number of surrounding cells
					s_cells = self.get_surrounding_cells(rand_wall)

					if (s_cells < 2):
						# Denote the new path
						self.maze[rand_wall[0]][rand_wall[1]] = self.cell_token

						# Mark the new walls
						# Upper cell
						if (rand_wall[0] != 0):
							if (self.maze[rand_wall[0]-1][rand_wall[1]] != self.cell_token):
								self.maze[rand_wall[0]-1][rand_wall[1]] = self.wall_token
							if ([rand_wall[0]-1, rand_wall[1]] not in walls):
								walls.append([rand_wall[0]-1, rand_wall[1]])


						# Bottom cell
						if (rand_wall[0] != self.height-1):
							if (self.maze[rand_wall[0]+1][rand_wall[1]] != self.cell_token):
								self.maze[rand_wall[0]+1][rand_wall[1]] = self.wall_token
							if ([rand_wall[0]+1, rand_wall[1]] not in walls):
								walls.append([rand_wall[0]+1, rand_wall[1]])

						# Leftmost cell
						if (rand_wall[1] != 0):	
							if (self.maze[rand_wall[0]][rand_wall[1]-1] != self.cell_token):
								self.maze[rand_wall[0]][rand_wall[1]-1] = self.wall_token
							if ([rand_wall[0], rand_wall[1]-1] not in walls):
								walls.append([rand_wall[0], rand_wall[1]-1])
					

					# Delete wall
					for wall in walls:
						if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
							walls.remove(wall)

					continue

			# Check if it is an upper wall
			if (rand_wall[0] != 0):
				if (self.maze[rand_wall[0]-1][rand_wall[1]] == self.unvisited_token and self.maze[rand_wall[0]+1][rand_wall[1]] == self.cell_token):

					s_cells = self.get_surrounding_cells(rand_wall)
					if (s_cells < 2):
						# Denote the new path
						self.maze[rand_wall[0]][rand_wall[1]] = self.cell_token

						# Mark the new walls
						# Upper cell
						if (rand_wall[0] != 0):
							if (self.maze[rand_wall[0]-1][rand_wall[1]] != self.cell_token):
								self.maze[rand_wall[0]-1][rand_wall[1]] = self.wall_token
							if ([rand_wall[0]-1, rand_wall[1]] not in walls):
								walls.append([rand_wall[0]-1, rand_wall[1]])

						# Leftmost cell
						if (rand_wall[1] != 0):
							if (self.maze[rand_wall[0]][rand_wall[1]-1] != self.cell_token):
								self.maze[rand_wall[0]][rand_wall[1]-1] = self.wall_token
							if ([rand_wall[0], rand_wall[1]-1] not in walls):
								walls.append([rand_wall[0], rand_wall[1]-1])

						# Rightmost cell
						if (rand_wall[1] != self.width-1):
							if (self.maze[rand_wall[0]][rand_wall[1]+1] != self.cell_token):
								self.maze[rand_wall[0]][rand_wall[1]+1] = self.wall_token
							if ([rand_wall[0], rand_wall[1]+1] not in walls):
								walls.append([rand_wall[0], rand_wall[1]+1])

					# Delete wall
					for wall in walls:
						if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
							walls.remove(wall)

					continue

			# Check the bottom wall
			if (rand_wall[0] != self.height-1):
				if (self.maze[rand_wall[0]+1][rand_wall[1]] == self.unvisited_token and self.maze[rand_wall[0]-1][rand_wall[1]] == self.cell_token):

					s_cells = self.get_surrounding_cells(rand_wall)
					if (s_cells < 2):
						# Denote the new path
						self.maze[rand_wall[0]][rand_wall[1]] = self.cell_token

						# Mark the new walls
						if (rand_wall[0] != self.height-1):
							if (self.maze[rand_wall[0]+1][rand_wall[1]] != self.cell_token):
								self.maze[rand_wall[0]+1][rand_wall[1]] = self.wall_token
							if ([rand_wall[0]+1, rand_wall[1]] not in walls):
								walls.append([rand_wall[0]+1, rand_wall[1]])
						if (rand_wall[1] != 0):
							if (self.maze[rand_wall[0]][rand_wall[1]-1] != self.cell_token):
								self.maze[rand_wall[0]][rand_wall[1]-1] = self.wall_token
							if ([rand_wall[0], rand_wall[1]-1] not in walls):
								walls.append([rand_wall[0], rand_wall[1]-1])
						if (rand_wall[1] != self.width-1):
							if (self.maze[rand_wall[0]][rand_wall[1]+1] != self.cell_token):
								self.maze[rand_wall[0]][rand_wall[1]+1] = self.wall_token
							if ([rand_wall[0], rand_wall[1]+1] not in walls):
								walls.append([rand_wall[0], rand_wall[1]+1])

					# Delete wall
					for wall in walls:
						if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
							walls.remove(wall)


					continue

			# Check the right wall
			if (rand_wall[1] != self.width-1):
				if (self.maze[rand_wall[0]][rand_wall[1]+1] == self.unvisited_token and self.maze[rand_wall[0]][rand_wall[1]-1] == self.cell_token):

					s_cells = self.get_surrounding_cells(rand_wall)
					if (s_cells < 2):
						# Denote the new path
						self.maze[rand_wall[0]][rand_wall[1]] = self.cell_token

						# Mark the new walls
						if (rand_wall[1] != self.width-1):
							if (self.maze[rand_wall[0]][rand_wall[1]+1] != self.cell_token):
								self.maze[rand_wall[0]][rand_wall[1]+1] = self.wall_token
							if ([rand_wall[0], rand_wall[1]+1] not in walls):
								walls.append([rand_wall[0], rand_wall[1]+1])
						if (rand_wall[0] != self.height-1):
							if (self.maze[rand_wall[0]+1][rand_wall[1]] != self.cell_token):
								self.maze[rand_wall[0]+1][rand_wall[1]] = self.wall_token
							if ([rand_wall[0]+1, rand_wall[1]] not in walls):
								walls.append([rand_wall[0]+1, rand_wall[1]])
						if (rand_wall[0] != 0):	
							if (self.maze[rand_wall[0]-1][rand_wall[1]] != self.cell_token):
								self.maze[rand_wall[0]-1][rand_wall[1]] = self.wall_token
							if ([rand_wall[0]-1, rand_wall[1]] not in walls):
								walls.append([rand_wall[0]-1, rand_wall[1]])

					# Delete wall
					for wall in walls:
						if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
							walls.remove(wall)

					continue

			# Delete the wall from the list anyway
			for wall in walls:
				if (wall[0] == rand_wall[0] and wall[1] == rand_wall[1]):
					walls.remove(wall)
			
		# Mark the remaining unvisited cells as walls
		for i in range(0, self.height):
			for j in range(0, self.width):
				if (self.maze[i][j] == self.unvisited_token):
					self.maze[i][j] = self.wall_token

		# Set entrance and exit
		for i in range(0, self.width):
			if (self.maze[1][i] == self.cell_token):
				self.maze[0][i] = self.cell_token
				break

		for i in range(self.width-1, 0, -1):
			if (self.maze[self.height-2][i] == self.cell_token):
				self.maze[self.height-1][i] = self.cell_token
				break

	def print_maze(self):
		for height in range(0, self.height):
			for width in range(0, self.width):
				if (self.maze[height][width] == self.unvisited_token):
					print(Fore.WHITE + str(self.maze[height][width]), end=" ")
				elif (self.maze[height][width] == self.cell_token):
					print(Fore.GREEN + str(self.maze[height][width]), end=" ")
				else:
					print(Fore.RED + str(self.maze[height][width]), end=" ")			
			print('\n')

if __name__ == "__main__":
	init() # Initialize colorama
	maze = Maze()

	maze.generate_maze()
	maze.print_maze()
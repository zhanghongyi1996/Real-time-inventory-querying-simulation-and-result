'''
Warehouse wall creater for ploting and avoidance properties. 

Used in the_swarm.py to create the warehouse walls for the simulation

Code authored by Elliot Hogg
This version was edited down by Emma Milner
'''
import numpy as np
	
class make_wall(object):
    def __init__(self):
        self.start = np.array([0,0])
        self.end = np.array([0,0])
        self.width = 1
        self.hitbox = []
		
class make_box(object):
    '''
    Class which contains definitions for building a bounding box.
    '''
    def __init__(self, h, w, mid_point):
        self.height = h
        self.width = w
        self.walls = []

        self.walls.append(make_wall())
        self.walls[0].start = [mid_point[0]-(0.5*w), mid_point[1]+(0.5*h)]; self.walls[0].end = [mid_point[0]+(0.5*w), mid_point[1]+(0.5*h)]
        self.walls.append(make_wall())
        self.walls[1].start = [mid_point[0]-(0.5*w), mid_point[1]-(0.5*h)]; self.walls[1].end = [mid_point[0]+(0.5*w), mid_point[1]-(0.5*h)]
        self.walls.append(make_wall())
        self.walls[2].start = [mid_point[0]-(0.5*w), mid_point[1]+(0.5*h)]; self.walls[2].end = [mid_point[0]-(0.5*w), mid_point[1]-(0.5*h)]
        self.walls.append(make_wall())
        self.walls[3].start = [mid_point[0]+(0.5*w), mid_point[1]+(0.5*h)]; self.walls[3].end = [mid_point[0]+(0.5*w), mid_point[1]-(0.5*h)]
   
		
class map():
	def __init__(self):
		self.obstacles = [] # contains a list of all walls that make up an enviornment
		self.walls = np.array([]) # same as obsticales variable but as a numpy array
		self.wallh = np.array([]) # a list of only horizontal walls
		self.wallv = np.array([]) # a list of only vertical walls
		self.planeh = np.array([]) # a list of horizontal avoidance planes formed by walls
		self.planev = np.array([]) # a list of horizontal vertical planes formed by walls
	def gen(self):
		self.walls = np.zeros((2*len(self.obstacles), 2))
		self.wallh = np.zeros((2*len(self.obstacles), 2))
		self.wallv = np.zeros((2*len(self.obstacles), 2))
		self.planeh = np.zeros(len(self.obstacles))
		self.planev = np.zeros(len(self.obstacles))
		self.limh = np.zeros((len(self.obstacles), 2))
		self.limv = np.zeros((len(self.obstacles), 2))

		for n in range(0, len(self.obstacles)):
			if self.obstacles[n].start[0] == self.obstacles[n].end[0]:
				self.wallv[2*n] = np.array([self.obstacles[n].start[0], self.obstacles[n].start[1]])
				self.wallv[2*n+1] = np.array([self.obstacles[n].end[0], self.obstacles[n].end[1]])

				self.planev[n] = self.wallv[2*n][0]
				self.limv[n] = np.array([np.min([self.obstacles[n].start[1], self.obstacles[n].end[1]])-0.5, np.max([self.obstacles[n].start[1], self.obstacles[n].end[1]])+0.5])

			# if wall is horizontal
			if self.obstacles[n].start[1] == self.obstacles[n].end[1]:
				self.wallh[2*n] = np.array([self.obstacles[n].start[0], self.obstacles[n].start[1]])
				self.wallh[2*n+1] = np.array([self.obstacles[n].end[0], self.obstacles[n].end[1]])

				self.planeh[n] = self.wallh[2*n][1]
				self.limh[n] = np.array([np.min([self.obstacles[n].start[0], self.obstacles[n].end[0]])-0.5, np.max([self.obstacles[n].start[0], self.obstacles[n].end[0]])+0.5])

			self.walls[2*n] = np.array([self.obstacles[n].start[0], self.obstacles[n].start[1]])
			self.walls[2*n+1] = np.array([self.obstacles[n].end[0], self.obstacles[n].end[1]])
	
	def warehouse_map(self,width,height):
		box = make_box(height, width, [width/2, height/2]); [self.obstacles.append(box.walls[x]) for x in range(0, len(box.walls))]
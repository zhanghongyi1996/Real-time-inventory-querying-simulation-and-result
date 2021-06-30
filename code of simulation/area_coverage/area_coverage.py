import numpy as np
import math
import random
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
import scipy
from scipy.spatial.distance import cdist, pdist, euclidean
import warehouse
import pickle
import sys
import os


radius = 12.5 # Radius of single agent (half of 25)
width = 1000 # Width of warehouse (100)
height = 1000 # Height (depth) of warehouse (100)
speed = 2 # Agent speed (0.5)
repulsion_distance = radius/2# Distance at which repulsion is first felt (3)
R_rob = 15 # repulsion 'forces'/influence factors for robots-robots
R_wall = 30 # repulsion 'forces'/influence factors for robots-walls


marker_size = 300*0.5/20 #diameter


class square():
      def __init__(self):
          self.square_node = np.zeros((height, width))
          self.square_position_x = np.zeros((height,width))
          self.square_position_y = np.zeros((height,width))
          for m in range(0, height):
              for n in range(0, width):
                  self.square_position_x[m][n] = n + 0.5
                  self.square_position_y[m][n] = m + 0.5
class swarm():
      def __init__(self,num_agents,scan_scope):
             self.speed = speed # Agent speed
             self.num_agents = num_agents # Swarm size
             self.heading = 0.0314*np.random.randint(-100,100,self.num_agents) # initial heading for all robots is randomly chosen
             self.rob_c = np.random.randint(radius*2,width-radius*2,(self.num_agents,2)) # rob_c is the centre coordinate of the agent which starts at a random position within the warehouse
             self.counter = 0 # time starts at 0s or time step = 0
             self.rob_d = np.zeros((self.num_agents,2)) # robot centre cooridinate deviation (how much the robot moves in one time step)
             self.original_position = np.zeros((self.num_agents,2))
             self.current_position = np.zeros((self.num_agents,2))
             self.cleaner_counter = 0
             self.scan_range = scan_scope
             self.cleaner_counter_box_in_area = 0
      def iterate(self,square):
             random_walk(self)
             self.original_position = self.rob_c
             self.rob_c = self.rob_c + self.rob_d
             self.current_position = self.rob_c
             a1 = []
             b1 = []
             b2 = []
             a2 = []
             c1 = []
             c2= []
             for n in range(0, self.num_agents):
                 a1.append(self.rob_d[n][1] / self.rob_d[n][0])
                 a2.append(-self.rob_d[n][0] / self.rob_d[n][1])
             for n in range(0, self.num_agents):
                 b1.append(self.rob_c[n][1] - a1[n] * self.rob_c[n][0]+radius/(self.rob_d[n][0]/(self.rob_d[n][0]**2+self.rob_d[n][1]**2)**0.5))
                 b2.append(self.rob_c[n][1] - a1[n] * self.rob_c[n][0]-radius/(self.rob_d[n][0]/(self.rob_d[n][0]**2+self.rob_d[n][1]**2)**0.5))
                 c1.append(self.original_position[n][1] - a2[n] * self.original_position[n][0])
                 c2.append(self.current_position[n][1] - a2[n] * self.current_position[n][0])
             for n in range(0, self.num_agents):
                 for i in range(max(math.ceil(min(self.original_position[n][1],self.current_position[n][1])-self.scan_range),0),min(math.floor(max(self.original_position[n][1],self.current_position[n][1])+self.scan_range),height)):
                     for j in range(max(math.ceil(min(self.original_position[n][0],self.current_position[n][0])-self.scan_range),0),min(math.floor(max(self.original_position[n][0],self.current_position[n][0])+self.scan_range),width)):
                         if square.square_node[i][j] == 0:
                            if ((square.square_position_y[i][j]-a1[n] * square.square_position_x[i][j] - b1[n])*(square.square_position_y[i][j]-a1[n] * square.square_position_x[i][j] - b2[n]) < 0 and (square.square_position_y[i][j]-a2[n] * square.square_position_x[i][j] - c1[n])*(square.square_position_y[i][j]-a2[n] * square.square_position_x[i][j] - c2[n]) < 0) or ((square.square_position_x[i][j]-self.original_position[n][0])**2+(square.square_position_y[i][j]-self.original_position[n][1])**2)**0.5 < self.scan_range or ((square.square_position_x[i][j]-self.current_position[n][0])**2+(square.square_position_y[i][j]-self.current_position[n][1])**2)**0.5 < self.scan_range:
                                square.square_node[i][j] = 1
                                self.cleaner_counter = self.cleaner_counter + 1
                                if i >= 25 and i <= 974 and j >= 25 and j <=974:
                                    self.cleaner_counter_box_in_area =self.cleaner_counter_box_in_area + 1
                            else:
                                continue
                         else:
                             continue

def random_walk(swarm):
    swarm.counter += 1 # time step forwards 1s
    # Add noise to the heading
    noise = 0.01*np.random.randint(-50,50,(swarm.num_agents))
    swarm.heading = noise + swarm.heading

    heading_x = 1*np.cos(swarm.heading) # move in x
    heading_y = 1*np.sin(swarm.heading) # move in y
    F_heading = -np.array([[heading_x[n], heading_y[n]] for n in range(0,swarm.num_agents)])  # influence on the robot's movement based on the noise added to the heading
    r = repulsion_distance
    agent_distance = cdist(swarm.rob_c, swarm.rob_c)  # distance between all the agents to all the agents
    proximity_to_robots = swarm.rob_c[:,:,np.newaxis]-swarm.rob_c.T[np.newaxis,:,:] #calculate the direction of robots to robots
    F_agent = R_rob * r * np.exp(-agent_distance / r)[:, np.newaxis, :] * proximity_to_robots / (swarm.num_agents - 1)
    F_agent = np.sum(F_agent, axis=0).T  # sum the repulsion vectors

    # Force on agent due to proximity to walls
    F_wall_avoidance = avoidance(swarm.rob_c, swarm.map)
    # Repulsion vectors added together
    F_agent = F_wall_avoidance + F_heading + F_agent
    F_x = F_agent.T[0]  # Repulsion vector in x
    F_y = F_agent.T[1]  # in y

    # New movement due to repulsion vectors
    new_heading = np.arctan2(F_y, F_x)  # new heading due to repulsions
    move_x = swarm.speed * np.cos(new_heading)  # Movement in x
    move_y = swarm.speed * np.sin(new_heading)  # Movement in y

    # Total change in movement of agent (robot deviation)
    swarm.rob_d = -np.array([[move_x[n], move_y[n]] for n in range(0, swarm.num_agents)])
    return swarm.rob_d



def avoidance(rob_c, map):  # input the agent positions array and the warehouse map
    num_agents = len(rob_c)  # num_agents is number of agents according to position array
    ## distance from agents to walls ##
    # distance from the vertical walls to your agent (horizontal distance between x coordinates)
    difference_in_x = np.array([map.planeh - rob_c[n][1] for n in range(num_agents)])
    # distance from the horizontal walls to your agent (vertical distance between y coordinates)
    difference_in_y = np.array([map.planev - rob_c[n][0] for n in range(num_agents)])

    # x coordinates of the agent's centre coordinate
    agentsx = rob_c.T[0]
    # y coordinates
    agentsy = rob_c.T[1]

    ## Are the agents within the limits of the warehouse?
    x_lower_wall_limit = agentsx[:, np.newaxis] >= map.limh.T[0]  # limh is for horizontal walls. x_lower is the bottom of the square
    x_upper_wall_limit = agentsx[:, np.newaxis] <= map.limh.T[1]  # x_upper is the top bar of the warehouse square
    # Interaction combines the lower and upper limit information to give a TRUE or FALSE value to the agents depending on if it is IN/OUT the warehouse boundaries
    interaction = x_upper_wall_limit * x_lower_wall_limit

    # Fy is repulsion vector on the agent in y direction due to proximity to the horziontal walls
    # This equation was designed to be very high when the agent is close to the wall and close to 0 otherwise
    Fy = np.exp(-2 * abs(difference_in_x) + R_wall)
    # The repulsion vector is zero if the interaction is FALSE meaning that the agent is safely within the warehouse boundary
    Fy = Fy * difference_in_x * interaction

    # Same as x boundaries but now in y
    y_lower_wall_limit = agentsy[:, np.newaxis] >= map.limv.T[0]  # limv is vertical walls
    y_upper_wall_limit = agentsy[:, np.newaxis] <= map.limv.T[1]
    interaction = y_lower_wall_limit * y_upper_wall_limit
    Fx = np.exp(-2 * abs(difference_in_y) + R_wall)
    Fx = Fx * difference_in_y * interaction

    # For each agent the repulsion in x and y is the sum of the repulsion vectors from each wall
    Fx = np.sum(Fx, axis=1)
    Fy = np.sum(Fy, axis=1)
    # Combine to one vector variable
    F = np.array([[Fx[n], Fy[n]] for n in range(num_agents)])
    return F


class data:
    def __init__(self,num_agents,anim,limit,scan):
        self.num_agents = num_agents
        self.scan_scope = scan
        self.robots = swarm(self.num_agents,self.scan_scope)
        self.time = limit
        self.anim = anim
        self.counter = 0
        self.new_square = square()

        warehouse_map = warehouse.map()
        warehouse_map.warehouse_map(width,height)
        warehouse_map.gen()
        self.robots.map = warehouse_map

        self.data_collect()
    def data_collect(self):
        self.robots.iterate(self.new_square)
        if self.anim == False:
            while 1:
                  print("cover range is ",self.robots.cleaner_counter_box_in_area)
                  self.robots.iterate(self.new_square)
                  self.counter=self.counter+1
                  if self.robots.cleaner_counter_box_in_area == 1 * 950 * 950:
                      print("Whole process uses",self.counter,"s")
                      return self.counter
                      exit()
        if self.anim == True:
            self.ani()

    def ani(self):
        fig = plt.figure()
        ax = plt.axes(xlim=(0, width), ylim=(0, height))
        dot, = ax.plot([self.robots.rob_c[i, 0] for i in range(self.num_agents)],
                       [self.robots.rob_c[i, 1] for i in range(self.num_agents)],
                       'ko',
                       markersize=marker_size, fillstyle='none')

        plt.axis('square')
        plt.axis([0, width, 0, height])
        def animate(i):
            self.robots.iterate(self.new_square)
            dot.set_data([self.robots.rob_c[n, 0] for n in range(self.num_agents)],
                         [self.robots.rob_c[n, 1] for n in range(self.num_agents)])
            plt.title("Time is " + str(self.robots.counter) + "s")
            print("cover range is ", self.robots.cleaner_counter)
        anim = animation.FuncAnimation(fig, animate, frames=500, interval=0.1)

        plt.xlabel("Warehouse width (cm)")
        plt.ylabel("Warehouse height (cm)")
        plt.show()
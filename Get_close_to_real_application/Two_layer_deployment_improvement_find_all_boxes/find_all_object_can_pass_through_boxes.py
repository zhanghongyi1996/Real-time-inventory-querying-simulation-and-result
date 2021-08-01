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
#width = 500 # Width of warehouse (100)
#height = 500 # Height (depth) of warehouse (100)
speed = 2 # Agent speed (0.5)
repulsion_distance = radius/2# Distance at which repulsion is first felt (3)

R_rob = 15 # repulsion 'forces'/influence factors for robots-robots
R_wall = 30 # repulsion 'forces'/influence factors for robots-walls

marker_size = 250*0.5/20 #diameter

box_radius = radius # radius of the box is the same as the robots

R_box = 15 # repulsion 'forces'/influence factors for robots-boxes


class boxes():
      def __init__(self, number_of_boxes,width_box):
          self.num_boxes = number_of_boxes
          self.box_in_width = width_box
          self.box_c = np.random.randint(box_radius * 2, self.box_in_width - box_radius * 2, (self.num_boxes, 2))
          self.radius = box_radius
          self.label = list(range(self.num_boxes))
          self.all_target = self.label
class swarm():
      def __init__(self,num_agents,scan_scope,width_swarm,obstacle):
             self.swarm_in_width = width_swarm
             self.speed = speed # Agent speed
             self.num_agents = num_agents # Swarm size
             self.heading = 0.0314*np.random.randint(-100,100,self.num_agents) # initial heading for all robots is randomly chosen
             self.rob_c = np.random.randint(radius*2,self.swarm_in_width-radius*2,(self.num_agents,2)) # rob_c is the centre coordinate of the agent which starts at a random position within the warehouse
             self.counter = 0 # time starts at 0s or time step = 0
             self.rob_d = np.zeros((self.num_agents,2)) # robot centre cooridinate deviation (how much the robot moves in one time step)
             self.rob_scan = scan_scope # the scan range of the robot
             self.rob_collect = [[] for _ in range(num_agents)] # robots collect boxes information until now
             self.everyterm = [[] for _ in range(num_agents)] # robots collect information in every move
             self.unioninform = [] # collect all information in the robot swarm(include repeated information)
             self.last_heading = 0
             self.noise = np.zeros(self.num_agents)
             self.obstacle_feel = obstacle
      def iterate(self,boxes):
             random_walk(self,boxes)
             self.rob_c = self.rob_c + self.rob_d
             self.everyterm = infcollect(self, boxes)
             # update the collection information of each robot
             for n in range(0,self.num_agents):
                 self.rob_collect[n] = list(set(self.rob_collect[n]).union(set(self.everyterm[n])))
             self.unioninform = [i for item in self.rob_collect for i in item]

def random_walk(swarm,boxes):
    swarm.counter += 1 # time step forwards 1s
    # Add noise to the heading
    swarm.heading = swarm.heading + swarm.noise

    heading_x = 1*np.cos(swarm.heading) # move in x
    heading_y = 1*np.sin(swarm.heading) # move in y
    F_heading = np.array([[heading_x[n], heading_y[n]] for n in range(0,swarm.num_agents)])  # influence on the robot's movement based on the noise added to the heading
    r = repulsion_distance
    agent_distance = cdist(swarm.rob_c, swarm.rob_c)  # distance between all the agents to all the agents
    proximity_to_robots = swarm.rob_c[:,:,np.newaxis]-swarm.rob_c.T[np.newaxis,:,:] #calculate the direction of robots to robots
    F_agent = R_rob * r * np.exp(-agent_distance / r)[:, np.newaxis, :] * proximity_to_robots / (swarm.num_agents - 1)
    for i in range(swarm.num_agents):
        for j in range(swarm.num_agents):
            if agent_distance[i,j] > (swarm.obstacle_feel + radius +radius):
               F_agent[i, 0, j] = 0
               F_agent[i, 1, j] = 0
    F_agent = np.sum(F_agent, axis=0).T  # sum the repulsion vectors

    # Force on agent due to proximity to walls
    F_wall_avoidance = avoidance(swarm.rob_c, swarm.map, swarm.obstacle_feel)

    # Repulsion between boxes and robots
    # box_dist = cdist(boxes.box_c,swarm.rob_c)
    # proximity_to_boxes = boxes.box_c[:, :, np.newaxis] - swarm.rob_c.T[np.newaxis, :, :]
    # F_box = R_box * r * np.exp(-box_dist / r)[:, np.newaxis, :] * proximity_to_boxes / (swarm.num_agents - 1)
    # F_box = np.sum(F_box, axis=0).T  # sum the repulsion vectors due to boxes on the agents

    # Repulsion vectors added together
    F_agent = F_wall_avoidance + F_heading + F_agent #+ F_box
    F_x = F_agent.T[0]  # Repulsion vector in x
    F_y = F_agent.T[1]  # in y

    # New movement due to repulsion vectors
    new_heading = np.arctan2(F_y, F_x)  # new heading due to repulsions
    move_x = swarm.speed * np.cos(new_heading)  # Movement in x
    move_y = swarm.speed * np.sin(new_heading)  # Movement in y

    # Total change in movement of agent (robot deviation)
    swarm.rob_d = -np.array([[move_x[n], move_y[n]] for n in range(0, swarm.num_agents)])
    swarm.last_heading = new_heading
    for i in range(swarm.num_agents):
        if swarm.last_heading[i] == swarm.heading[i]:
            swarm.noise[i] = 0
        else:
            swarm.noise[i] = 0.01 * np.random.randint(-50, 50, (1))
    swarm.heading = swarm.last_heading

    return swarm.rob_d



def avoidance(rob_c, map, obstacle_feels):  # input the agent positions array and the warehouse map
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
    for i in range(num_agents):
        if abs(difference_in_x[i,0]) > (obstacle_feels + radius) and abs(difference_in_x[i,1]) > (obstacle_feels + radius):
           Fy[i] = 0
        if abs(difference_in_y[i,2]) > (obstacle_feels + radius) and abs(difference_in_y[i,3]) > (obstacle_feels + radius):
           Fx[i] = 0
    F = np.array([[Fx[n], Fy[n]] for n in range(num_agents)])
    return F

def infcollect(swarm,boxes):
    # calculate the vector of the robot and boxes
    vector_robot_box = swarm.rob_c[:, :, np.newaxis] - boxes.box_c.T[np.newaxis, :, :]
    # calculate the distance between the robot and boxes
    distance_robot_box = ((vector_robot_box ** 2)[:, 0, :] + (vector_robot_box ** 2)[:, 1, :]) ** 0.5
    # collect information in one term
    information_collection = [[] for _ in range(len(distance_robot_box))]
    #for i in range(len(distance_robot_box)):
        #for j in range(len(distance_robot_box[0])):
            #if distance_robot_box[i, j] <= swarm.rob_scan:
                #information_collection[i].append(j)
    object_find = np.argwhere(distance_robot_box <= swarm.rob_scan)
    for i in range(len(object_find)):
        information_collection[object_find[i,0]].append(object_find[i,1])
    return information_collection

class data:
    def __init__(self,num_agents,anim,limit,num_boxes,scan_scope,data_width,data_height,obstacles):
        self.data_in_width = data_width
        self.data_in_height = data_height
        self.obstacle_ff = obstacles
        self.scan_scope = scan_scope
        self.num_agents = num_agents
        self.num_boxes = num_boxes
        self.items = boxes(self.num_boxes,self.data_in_width)
        self.robots = swarm(self.num_agents,self.scan_scope,self.data_in_width,self.obstacle_ff)
        self.time = limit
        self.anim = anim
        self.repeat_information = 0

        warehouse_map = warehouse.map()
        warehouse_map.warehouse_map(self.data_in_width,self.data_in_height)
        warehouse_map.gen()
        self.robots.map = warehouse_map
        self.data_collect()

    def data_collect(self):
        self.robots.iterate(self.items)
        if self.anim == False:
            while self.robots.counter <= self.time:
                self.robots.iterate(self.items)
                if sorted(list(set(self.robots.unioninform).intersection(set(self.items.all_target)))) == self.items.all_target:
                    print('target customers want to get is', self.items.all_target)
                    print('all information collect in', self.robots.counter, 's')
                    print('information collection of each robot', self.robots.rob_collect)
                    #repeat_information = 0
                    self.repeat_information = len(self.robots.unioninform) - len(self.items.all_target)
                    print('number of repeat target information in whole robot swarm is', self.repeat_information)
                    print('union information is', self.robots.unioninform)
                    return self.robots.counter,self.repeat_information

            print('related information can not be completely collected in limited time')
            print('total collected information is',list(set(self.robots.unioninform).intersection(set(self.items.all_target))))

            for i in range(len(self.items.all_target)):
                self.repeat_information = self.repeat_information + len(np.argwhere(np.array(self.robots.unioninform) == self.items.all_target[i]))
            self.repeat_information = self.repeat_information - len(list(set(self.robots.unioninform).intersection(set(self.items.all_target))))
            print('number of repeated information is',self.repeat_information)
            return self.robots.counter,self.repeat_information

        if self.anim == True:
            self.ani()

    def ani(self):
        fig = plt.figure()
        ax = plt.axes(xlim=(0, self.data_in_width), ylim=(0, self.data_in_height))
        dot, = ax.plot([self.robots.rob_c[i, 0] for i in range(self.num_agents)],
                       [self.robots.rob_c[i, 1] for i in range(self.num_agents)],
                       'ko',
                       markersize=marker_size-1, fillstyle='none')
        box, = ax.plot([self.items.box_c[i, 0] for i in range(self.num_boxes)],
                       [self.items.box_c[i, 1] for i in range(self.num_boxes)], 'rs', markersize=marker_size - 3)

        plt.axis('square')
        plt.axis([0, self.data_in_width, 0, self.data_in_height])
        def animate(i):
            self.robots.iterate(self.items)
            dot.set_data([self.robots.rob_c[n, 0] for n in range(self.num_agents)],
                         [self.robots.rob_c[n, 1] for n in range(self.num_agents)])
            box.set_data([self.items.box_c[n, 0] for n in range(self.num_boxes)],
                         [self.items.box_c[n, 1] for n in range(self.num_boxes)])
            plt.title("Time is " + str(self.robots.counter) + "s")
            if self.robots.counter > self.time:
                print(self.robots.rob_collect)
                print(self.robots.everyterm)
                print('related target can not be found in time')
                exit()
            if  sorted(list(set(self.robots.unioninform).intersection(set(self.items.all_target)))) == self.items.all_target:
                print('target customers want to get is',self.items.all_target)
                print('all information collect in', self.robots.counter,'s')
                print('information collection of each robot',self.robots.rob_collect)
                for i in range(len(self.robots.unioninform)):
                    for j in range(len(self.items.all_target)):
                        if self.robots.unioninform[i] == self.items.all_target[j]:
                            self.repeat_information = self.repeat_information + 1
                self.repeat_information = self.repeat_information - len(list(set(self.robots.unioninform).intersection(set(self.items.all_target))))
                print('number of repeated information is', self.repeat_information)
                print('union information is',self.robots.unioninform)
                exit()
        anim = animation.FuncAnimation(fig, animate, frames=500, interval=0.1)

        plt.xlabel("Warehouse width (cm)")
        plt.ylabel("Warehouse height (cm)")
        plt.show()
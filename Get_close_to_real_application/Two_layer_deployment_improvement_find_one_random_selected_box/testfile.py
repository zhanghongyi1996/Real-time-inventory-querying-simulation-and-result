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
import find_one_object_can_pass_through_the_box

swarm_size = 10
ani = True
time_limit = 300000
num_boxes = 10
scan_range = 40
width = 1000
height = 1000
obstacle = 5
find_one_object_can_pass_through_the_box.data(swarm_size,ani,time_limit,num_boxes,scan_range,width,height,obstacle)
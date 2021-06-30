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
import find_all_object_can_pass_through_boxes

swarm_size = 30
ani = False
time_limit = 300000
num_boxes = 10
scan_range = 5
width = 1000
height = 1000
ani = True
find_all_object_can_pass_through_boxes.data(swarm_size,ani,time_limit,num_boxes,scan_range,width,height)
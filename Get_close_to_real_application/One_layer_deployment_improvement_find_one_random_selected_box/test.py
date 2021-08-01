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
import seaborn as sns
import product_finding

swarm_size = 10
ani = True
time_limit = 300000
num_boxes = 30
scan_range = 40
width = 1000
height = 1000
obstacle = 5
product_finding.data(swarm_size,ani,time_limit,num_boxes,scan_range,width,height,obstacle)



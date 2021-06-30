import area_coverage
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


number_agents = 10
anim = 0
limit = 1000000
trial = 15
scan_range = 2

variable_test_scan_scope = 30 # changing the scan scope
experiment_data_time = [[] for _ in range(variable_test_scan_scope)]
average_experiment_data_time = [[] for _ in range(variable_test_scan_scope)]

for scan_size in range(variable_test_scan_scope):
    for trial_size in range(trial):
        scan_scope = scan_range + 2 * scan_size
        data = area_coverage.data(number_agents, anim, limit, scan_scope).counter
        experiment_data_time[scan_size].append(data)
    average_experiment_data_time[scan_size] = np.mean(experiment_data_time[scan_size])
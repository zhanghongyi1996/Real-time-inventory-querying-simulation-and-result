import numpy as np
import math
import random
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
import scipy
from scipy.spatial.distance import cdist, pdist, euclidean
import seaborn as sns

# transform the txt file to matrix
def readdata(route):
    with open(route, "r") as f:
        content = f.read()

    lines = [line.split(" ") for line in content.split("\n")]
    lst = []
    for line in lines:
        lst += line
    print(lst)

    result = []
    for i in range(0, len(lst)):
        ccc = lst[i].replace(',', '')
        ddd = ccc.replace('[', '')
        eee = ddd.replace(']', '')
        result.append(int(eee))
    return result

# extract the data from the matrix
def press(route1):
    result1 = readdata(route1)
    variable_test = 30
    trial = 15
    experiment_data_time = [[] for _ in range(variable_test)]
    for scan_size in range(variable_test):
        for trial_size in range(trial):
            experiment_data_time[scan_size].append(result1[scan_size * trial + trial_size])
    print(experiment_data_time)
    return experiment_data_time
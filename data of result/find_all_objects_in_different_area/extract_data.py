import numpy as np
import math
import random
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
import scipy
from matplotlib.ticker import MultipleLocator
from scipy.spatial.distance import cdist, pdist, euclidean
import seaborn as sns
import pandas as pd

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
def extract_data_average(route1,route2):
    result1 = readdata(route1)
    result2 = readdata(route2)
    variable_test = 30
    trial = 15
    experiment_data_time = [[[] for _ in range(variable_test)] for _ in range(variable_test)]
    experiment_data_number_of_information = [[[] for _ in range(variable_test)] for _ in range(variable_test)]
    average_experiment_data_time = [[[] for _ in range(variable_test)] for _ in range(variable_test)]
    average_experiment_data_number_of_information = [[[] for _ in range(variable_test)] for _ in range(variable_test)]
    deviation_experiment_data_time = [[[] for _ in range(variable_test)] for _ in range(variable_test)]
    deviation_experiment_data_number_of_information = [[[] for _ in range(variable_test)] for _ in range(variable_test)]
    for rob_size in range(variable_test):
        for scan_size in range(variable_test):
            for trial_size in range(trial):
                experiment_data_time[rob_size][scan_size].append(result1[rob_size * variable_test * trial + scan_size * trial + trial_size])
                experiment_data_number_of_information[rob_size][scan_size].append(result2[rob_size * variable_test * trial + scan_size * trial + trial_size])
            average_experiment_data_time[rob_size][scan_size] = np.mean(experiment_data_time[rob_size][scan_size])
            average_experiment_data_number_of_information[rob_size][scan_size] = np.mean(experiment_data_number_of_information[rob_size][scan_size])
            deviation_experiment_data_time[rob_size][scan_size] = np.std(experiment_data_time[rob_size][scan_size])
            deviation_experiment_data_number_of_information[rob_size][scan_size] = np.std(experiment_data_number_of_information[rob_size][scan_size])
    print(average_experiment_data_time)
    average_experiment_data_time = np.array(average_experiment_data_time)
    deviation_experiment_data_time = np.array(deviation_experiment_data_time)
    deviation_experiment_data_number_of_information = np.array(deviation_experiment_data_number_of_information)
    data_time_max = max([i for item in average_experiment_data_time for i in item])
    data_information_max = max([i for item in average_experiment_data_number_of_information for i in item])
    performance_judgement = 0.5 * np.array(average_experiment_data_time) / data_time_max + 0.5 * np.array(average_experiment_data_number_of_information) / data_information_max
    return experiment_data_time,average_experiment_data_time,average_experiment_data_number_of_information,performance_judgement,deviation_experiment_data_time,deviation_experiment_data_number_of_information,experiment_data_number_of_information
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
import extract_data
from scipy import stats
from scipy.stats import chi2_contingency
from scipy.stats import chi2

variable_test = 30
trial = 90
name1 = 'raw_material_of_time_find_one_box_ten_meters_width.txt'
name2 = 'raw_material_of_duplicate_information_find_one_box_ten_meters_width.txt'
name3 = 'raw_material_of_time_find_one_box_twenty_meters_width.txt'
name4 = 'raw_material_of_duplicate_information_find_one_box_twenty_meters_width.txt'
# Read data
a = extract_data.extract_data_average(name1,name2)
b = extract_data.extract_data_average(name3,name4)
# extract the data of duplicate information rate
information1 = a[6]
information2 = b[6]
# used to load data of each trial
mean_trial1 = [[] for _ in range(trial)]
mean_trial2 = [[] for _ in range(trial)]
# used to put data of M_D
mean_3 = []
mean_4 = []
# used to load D_m
total1 = []
total2 = []
# calculate how many p-value is greater than 0.01
number_p_large = 0
# used to put Welch's t test result
s = [[[] for _ in range(variable_test)] for _ in range(variable_test)]
p_value = [[[] for _ in range(variable_test)] for _ in range(variable_test)]

# load data according to the order of trials
for k in range(trial):
    for i in range(variable_test):
        for j in range(variable_test):
            mean_trial1[k].append(information1[i][j][k])
            mean_trial2[k].append(information2[i][j][k])

# load D_m
for i in range(variable_test):
    for j in range(variable_test):
        total1.append(np.mean(information1[i][j]))
        total2.append(np.mean(information2[i][j]))

# Welch's t test for D_m
for i in range(variable_test):
    for j in range(variable_test):
        rvs1 = information1[i][j]
        rvs2 = information2[i][j]
        p_value[i][j] = stats.ttest_ind(rvs1,rvs2, equal_var = False)[1]
        if math.isnan(p_value[i][j]) == 1:
           p_value[i][j] = 1
        if p_value[i][j] >0.01:
           s[i][j] = 1
           number_p_large = number_p_large + 1
        else:
           s[i][j] = 0

# calculate M_D
for k in range(trial):
    mean_3.append(np.mean(mean_trial1[k]))
    mean_4.append(np.mean(mean_trial2[k]))

# Welch's t test for M_D
print(stats.ttest_ind(mean_3, mean_4, equal_var=False))
# calculate the standard deviation of D_m under different parameter pairs
print(np.std(total1),np.std(total2))

hist,bin = np.histogram(total1,bins = 30)
aaa = hist
hist,bin = np.histogram(total2,bins = 30)
bbb = hist

# calculate the JS divergence
M = 0.5 * (np.array(aaa/900)+np.array(bbb/900))
print(0.5 * scipy.stats.entropy(aaa/900,M) + 0.5 * scipy.stats.entropy(bbb/900,M))


print('mean of D_m in 10m x 10m area of finding one boxes is', np.mean(mean_3))
print('mean of D_m in 20m x 20m area of finding one boxes is', np.mean(mean_4))
print('standard deviation of D_m in 10m x 10m area of finding one boxes is', np.std(total1))
print('standard deviation of D_m in 20m x 20m area of finding one boxes is', np.std(total2))
print('Js divergence is', 0.5 * scipy.stats.entropy(aaa/900,M) + 0.5 * scipy.stats.entropy(bbb/900,M))
print('Number of large P value of D under different parameter pairs is', number_p_large)
print('P value of M_D is', stats.ttest_ind(mean_3, mean_4, equal_var=False)[1])
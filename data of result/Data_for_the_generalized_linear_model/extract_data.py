import numpy as np
import math
import random
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
import scipy
from scipy.spatial.distance import cdist, pdist, euclidean
import seaborn as sns
import read_data

trial = 15
variable_test = 30
c = []
d = []
deviation = []
deviation_information = []
tt = []
all = []
stdpd = []
for n in range(90):
    mean_trial1 = [[] for _ in range(trial)]
    mean_3 = []
    k = n + 9
    name1 = 'data_time_find_all_object' + str(int(k)) + '.txt'
    a = read_data.extract_data_average(name1)
    for pt in range(trial):
        for i in range(variable_test):
            for j in range(variable_test):
                mean_trial1[pt].append(a[2][i][j][pt]) # extract the data according to the order of trials
    for pt in range(trial):
        mean_3.append(np.mean(mean_trial1[pt])) # calculate M_T
    all.append(mean_3)
    c.append(np.mean(a[0])) # load T_m under different parameter pairs
    deviation.append(np.mean(a[1])) # calculate average of standard deviation of T of each parameter pair
    tt.append(int(k+1))

print(all)
for n in range(90):
    stdpd.append(np.std(all[n])) # calculate the standard deviation of M_T of 15 trials
print(stdpd)
print(c)

plt.figure(1)
plt.plot(tt,c)
plt.xlabel('Number of Boxes')
plt.ylabel('Average Time Consumption(seconds)')
plt.title('Average time of searching all information of boxes')

plt.figure(2)
plt.plot(tt,deviation)
plt.xlabel('Number of Boxes')
plt.ylabel('Average Standard Deviation of Time(seconds)')
plt.title('Average standard deviation of time of searching all boxes')

plt.figure(3)
plt.plot(tt,stdpd)
plt.show()
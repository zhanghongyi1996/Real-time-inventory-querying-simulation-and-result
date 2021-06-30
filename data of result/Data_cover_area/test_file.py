import numpy as np
import math
import random
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
import scipy
from scipy.spatial.distance import cdist, pdist, euclidean
import seaborn as sns
import compress_data
import pandas as pd

variable = 30
compress_result = [] # used to put original data set T_m
mean_compress = [[[] for _ in range(variable)] for _ in range(variable)] # used to put the mean of T_m under different parameter pair
# extract data
for n in range(30):
    k = n + 1
    name1 = 'Time_cover_100_percentage_box_area' + str(int(k)) + '.txt'
    compress_result.append(compress_data.press(name1))

# do average of 15 trials
for robot in range(variable):
    for scan in range(variable):
        mean_compress[robot][scan] = np.mean(compress_result[robot][scan])
        if mean_compress[robot][scan] > 8000:
           mean_compress[robot][scan] = 8000

# Plot the result
x_tick = []
y_tick = []
for n in range(30):
    x_tick.append(str(2+2*n))
    y_tick.append(str(10+10*n))
data_time_experiment={}
data_information_experiment={}
data_performance={}
data_time_deviation={}
data_information_deviation={}
for i in range(30):
    data_time_experiment[x_tick[i]] = np.array(mean_compress).T[i]
pd_data_time_experiment=pd.DataFrame(data_time_experiment,index=y_tick,columns=x_tick)
print(np.mean(compress_result))

plt.figure(1)
sns.heatmap(pd_data_time_experiment, cmap='rainbow')
plt.xlabel('Scan range',fontsize=20, color='k')
plt.ylabel('Robot',fontsize=20, color='k')
plt.title('Time Consumption')
plt.show()
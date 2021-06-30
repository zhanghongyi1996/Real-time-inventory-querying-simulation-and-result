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

c = []
d = []
deviation = []
deviation_information = []
tt = []
cmin = []
cmax = []
dmin = []
dmax = []

# calculate the average of T_m and D_m and related standard deviation of the result among different groups
for n in range(50):
    name1 = 'data_time_find_all_object' + str(int(n)) + '.txt'
    name2 = 'data_repeat_information_find_all_object' + str(int(n)) + '.txt'
    a = read_data.extract_data_average(name1,name2)
    c.append(np.mean(a[0]))
    d.append(np.mean(a[1]))
    deviation.append(np.std(a[0]))
    deviation_information.append(np.std(a[1]))
    if np.mean(a[0]) - np.std(a[0]) < 0:
       cmin.append(0)
    else:
       cmin.append(np.mean(a[0]) - np.std(a[0]))
    cmax.append(np.mean(a[0]) + np.std(a[0]))
    if np.mean(a[1]) - np.std(a[1]) < 0:
       dmin.append(0)
    else:
       dmin.append(np.mean(a[1]) - np.std(a[1]))
    dmax.append(np.mean(a[1]) + np.std(a[1]))
    tt.append(int(n+1))

print(c)
print(d)

plt.figure(1)
plt.subplot(121)
data_points = 50
x = np.linspace(1, data_points, num=data_points)
plt.plot(tt, c, linewidth=2., label='Average time taken', color='k')
plt.fill_between(x,cmin,cmax,facecolor='lightgrey',label='Standard deviation of average time')
plt.xlabel('Number of boxes',fontsize=20, color='k')
plt.ylabel('Time consumption(seconds)',fontsize=20, color='k')
plt.title('Average time of different groups',fontsize=15)
plt.grid(True)
plt.legend(loc='upper left')

plt.subplot(122)
x = np.linspace(1, data_points, num=data_points)
plt.plot(tt, d, linewidth=2., label='Average duplicate Information', color='k')
plt.fill_between(x,dmin,dmax,facecolor='lightgrey',label='Standard deviation of average duplicate information')
plt.xlabel('Number of boxes',fontsize=20, color='k')
plt.ylabel('Duplicate information rate',fontsize=20, color='k')
plt.title('Average duplicate information of different groups',fontsize=15)
plt.grid(True)
plt.legend(loc='upper left')

plt.show()
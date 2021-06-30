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

# calculate the average of T_m and D_m and related average standard deviation of the result within the group
for n in range(50):
    name1 = 'data_time_find_all_object' + str(int(n)) + '.txt'
    name2 = 'data_repeat_information_find_all_object' + str(int(n)) + '.txt'
    a = read_data.extract_data_average(name1,name2)
    c.append(np.mean(a[0]))
    d.append(np.mean(a[1]))
    deviation.append(np.mean(a[3]))
    deviation_information.append(np.mean(a[4]))
    cmin.append(np.mean(a[0]) - np.mean(a[3]))
    cmax.append(np.mean(a[0]) + np.mean(a[3]))
    dmin.append(np.mean(a[1]) - np.mean(a[4]))
    dmax.append(np.mean(a[1]) + np.mean(a[4]))
    tt.append(int(n+1))

print(c)
print(d)

plt.figure(1)
plt.subplot(121)
data_points = 50
x = np.linspace(1, data_points, num=data_points)
plt.plot(tt, c, linewidth=2., label='Average time taken', color='k')
plt.fill_between(x,cmin,cmax,facecolor='lightgrey',label='Average standard deviation of time')
plt.xlabel('Number of boxes',fontsize=20, color='k')
plt.ylabel('Average Time consumption(seconds)',fontsize=20, color='k')
plt.title('Time taken to find all boxes in the warehouse',fontsize=15)
plt.grid(True)
plt.legend(loc='upper left')

plt.subplot(122)
x = np.linspace(1, data_points, num=data_points)
plt.plot(tt, d, linewidth=2., label='Average duplicate Information rate', color='k')
plt.fill_between(x,dmin,dmax,facecolor='lightgrey',label='Average standard deviation of duplicate information')
plt.xlabel('Number of boxes',fontsize=20, color='k')
plt.ylabel('Duplicate information rate',fontsize=20, color='k')
plt.title('Duplicate information rate of finding all boxes',fontsize=15)
plt.grid(True)
plt.legend(loc='upper left')

plt.show()

import numpy as np
import math
import random
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
import scipy
from scipy.spatial.distance import cdist, pdist, euclidean
import seaborn as sns
import pandas as pd

# Read data
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

# Read and plot the result
result1 = readdata('data_time_find_all_object_20m49.txt')
result2 = readdata('data_repeat_information_find_all_object_20m49.txt')
variable_test = 30
trial = 15
experiment_data_time=[[[] for _ in range(variable_test)] for _ in range(variable_test)]
experiment_data_number_of_information=[[[] for _ in range(variable_test)] for _ in range(variable_test)]
average_experiment_data_time=[[[] for _ in range(variable_test)] for _ in range(variable_test)]
average_experiment_data_number_of_information=[[[] for _ in range(variable_test)] for _ in range(variable_test)]
deviation_experiment_data_time=[[[] for _ in range(variable_test)] for _ in range(variable_test)]
deviation_experiment_data_number_of_information=[[[] for _ in range(variable_test)] for _ in range(variable_test)]
for rob_size in range(variable_test):
    for scan_size in range(variable_test):
        for trial_size in range(trial):
            experiment_data_time[rob_size][scan_size].append(result1[rob_size * variable_test * trial + scan_size * trial + trial_size])
            experiment_data_number_of_information[rob_size][scan_size].append(result2[rob_size * variable_test * trial + scan_size * trial + trial_size])
        average_experiment_data_time[rob_size][scan_size]=np.mean(experiment_data_time[rob_size][scan_size])
        average_experiment_data_number_of_information[rob_size][scan_size]=np.mean(experiment_data_number_of_information[rob_size][scan_size])
        deviation_experiment_data_time[rob_size][scan_size] = np.std(experiment_data_time[rob_size][scan_size])
        deviation_experiment_data_number_of_information[rob_size][scan_size] = np.std(experiment_data_number_of_information[rob_size][scan_size])
print(average_experiment_data_time)
average_experiment_data_time = np.array(average_experiment_data_time)
average_experiment_data_number_of_information = np.array(average_experiment_data_number_of_information)
average_experiment_data_time[average_experiment_data_time > 2000] = 2000
deviation_experiment_data_time = np.array(deviation_experiment_data_time)
deviation_experiment_data_time[deviation_experiment_data_time > 2000] = 2000
data_time_max=max([i for item in average_experiment_data_time for i in item])
data_information_max=max([i for item in average_experiment_data_number_of_information for i in item])
performance_judgement=0.5*np.array(average_experiment_data_time)/data_time_max+0.5*np.array(average_experiment_data_number_of_information)/data_information_max

x_tick = []
y_tick = []
for n in range(30):
    x_tick.append(str(2+2*n))
    y_tick.append(str(40+40*n))
data_time_experiment={}
data_information_experiment={}
data_performance={}
data_time_deviation={}
data_information_deviation={}
for i in range(30):
    data_time_experiment[x_tick[i]] = np.array(average_experiment_data_time).T[i]
    data_information_experiment[x_tick[i]] = np.array(average_experiment_data_number_of_information).T[i]
    data_performance[x_tick[i]] = np.array(performance_judgement).T[i]
    data_time_deviation[x_tick[i]] = np.array(deviation_experiment_data_time).T[i]
    data_information_deviation[x_tick[i]] = np.array(deviation_experiment_data_number_of_information).T[i]
pd_data_time_experiment=pd.DataFrame(data_time_experiment,index=y_tick,columns=x_tick)
pd_data_information_experiment=pd.DataFrame(data_information_experiment,index=y_tick,columns=x_tick)
pd_data_performance=pd.DataFrame(data_performance,index=y_tick,columns=x_tick)
pd_data_time_deviation=pd.DataFrame(data_time_deviation,index=y_tick,columns=x_tick)
pd_data_information_deviation=pd.DataFrame(data_information_deviation,index=y_tick,columns=x_tick)

plt.figure(1)
plt.subplots_adjust(wspace =0.3, hspace =0)
plt.subplot(131)
sns.heatmap(pd_data_time_experiment, cmap='Reds')
plt.xlabel('Scan range',fontsize=20, color='k')
plt.ylabel('Robot',fontsize=20, color='k')
plt.title('Time Consumption')

plt.subplot(132)
sns.heatmap(pd_data_information_experiment, cmap='Reds')
plt.xlabel('Scan range',fontsize=20, color='k')
plt.ylabel('Robot',fontsize=20, color='k')
plt.title('Duplicate information rate')

plt.subplot(133)
sns.heatmap(pd_data_performance, cmap='Reds')
plt.xlabel('Scan range',fontsize=20, color='k')
plt.ylabel('Robot',fontsize=20, color='k')
plt.title('Performance')

plt.figure(2)
plt.subplots_adjust(wspace =0.2, hspace =0)
plt.subplot(121)
sns.heatmap(pd_data_time_deviation, cmap='Reds')
plt.xlabel('Scan range',fontsize=20, color='k')
plt.ylabel('Robot',fontsize=20, color='k')
plt.title('Standard deviation of time consumption')

plt.subplot(122)
sns.heatmap(pd_data_information_deviation, cmap='Reds')
plt.xlabel('Scan range',fontsize=20, color='k')
plt.ylabel('Robot',fontsize=20, color='k')
plt.title('Standard deviation of duplicate information rate')

plt.show()
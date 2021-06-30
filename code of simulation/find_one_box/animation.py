import find_one_object_can_pass_through_the_box
import numpy as np
import math
import matplotlib.pyplot as plt
import seaborn as sns


width_size = 0
basic_swarm_size = 10 # number of agents in the swarm
basic_num_boxes = 10 # number of boxes in simulation
basic_width = 1000 # Width of warehouse (100)
basic_height = 1000 # Height (depth) of warehouse (100)
basic_scan_scope = 1 # initial scan scope of the robots
time_limit = 600000 # how long to give the robots to complete the task
ani = False # Do you want to generate an animation of the behaviour?

variable_test_scan_scope = 30 # changing the scan scope
variable_test_robots = 50 # one change ten robots



experiment_data_time = [[[] for _ in range(variable_test_scan_scope)] for _ in range(variable_test_robots)]
experiment_data_number_of_information = [[[] for _ in range(variable_test_scan_scope)] for _ in range(variable_test_robots)]
average_experiment_data_time = [[[] for _ in range(variable_test_scan_scope)] for _ in range(variable_test_robots)]
average_experiment_data_number_of_information = [[[] for _ in range(variable_test_scan_scope)] for _ in range(variable_test_robots)]
trial = 30

for robot_size in range(variable_test_robots):
    for scan_size in range(variable_test_scan_scope):
        for trial_size in range(trial):
            width = basic_width + 500 * width_size
            height = basic_height + 500 * width_size
            num_boxes = basic_num_boxes
            swarm_size = basic_swarm_size + robot_size
            scan_range = basic_scan_scope + scan_size
            data = find_one_object_can_pass_through_the_box.data(swarm_size,ani,time_limit,num_boxes,scan_range,width,height)
            experiment_data_number_of_information[robot_size][scan_size].append(data.repeat_information)
            experiment_data_time[robot_size][scan_size].append(data.robots.counter)
        average_experiment_data_time[robot_size][scan_size]=np.mean(experiment_data_time[robot_size][scan_size])
        average_experiment_data_number_of_information[robot_size][scan_size]=np.mean(experiment_data_number_of_information[robot_size][scan_size])

print(experiment_data_time,experiment_data_number_of_information)
print(average_experiment_data_time,average_experiment_data_number_of_information)
# data_time_max=max([i for item in average_experiment_data_time for i in item])
# data_information_max=max([i for item in average_experiment_data_number_of_information for i in item])
# print(np.array(average_experiment_data_time)/data_time_max)
# print(np.array(average_experiment_data_number_of_information)/data_information_max)
# performance_judgement=0.5*np.array(average_experiment_data_time)/data_time_max+0.5*np.array(average_experiment_data_number_of_information)/data_information_max
# print(performance_judgement)


file=open('data_time_find_one_object.txt','w')
file.write(str(experiment_data_time));
file.close()

file=open('data_repeat_information_find_one_object.txt','w')
file.write(str(experiment_data_number_of_information));
file.close()

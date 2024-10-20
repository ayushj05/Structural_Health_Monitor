import numpy as np
import matplotlib.pyplot as plt

t_T_list = []
t_A_list = []
acc_x_list = []
acc_y_list = []
acc_z_list = []
x_mean = 0
y_mean = 0
z_mean = 0
temp = 0

with open("/path/to/SD_Card/Data.txt", 'r') as f:
    
    lines = f.readlines()

    time_to_add = 0

    prev_time = float(lines[3][2:lines[3].find(',', 0)])

    temp = float(lines[1][2:])
    print("Temp =", temp)

    for i in range(3, len(lines)):
        if lines[i][0] == 'T':
            time_to_add += 90
            temp = float(lines[1][2:])
            print("Temp =", temp)
            continue
        elif lines[i][0] != 't':
            continue

        sec_idx = lines[i].find(',', 0)
        time_ = float(lines[i][2:sec_idx]) + time_to_add

        t_A_list.append(time_)

        lines[i] = lines[i][sec_idx + 3:]
        comma_idx = lines[i].find(',')
        acc_x_list.append(float(lines[i][0:comma_idx]))
        x_mean += float(lines[i][0:comma_idx])
        
        lines[i] = lines[i][comma_idx + 1:]
        comma_idx = lines[i].find(',')
        acc_z_list.append(-float(lines[i][0:comma_idx]))
        z_mean -= float(lines[i][0:comma_idx])
        
        acc_y_list.append(float(lines[i][comma_idx + 1:]))
        y_mean += float(lines[i][comma_idx + 1:])

# Plotting
plt.figure()
plt.plot(t_A_list, acc_x_list, label='acc_x')
plt.plot(t_A_list, acc_y_list, label='acc_y')
plt.plot(t_A_list, acc_z_list, label='acc_z')
plt.xlabel('Time (in seconds)')
plt.ylabel('Acceleration (in mg)')
plt.legend()
plt.show()
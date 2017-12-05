#!/usr/bin/env python3
'''This modules graphes and collects antenna data'''

from RobotCommunicatorDebugging import RobotCommunicatorDebugging
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import scipy.io
import time
mc = RobotCommunicatorDebugging()

POINTS_NUMBER = 50

data_frame = mc.peek_motor_info()  # junk data for data structure
mc.set_control_loop_mode("open_loop")
for key in data_frame:
    data_frame[key] = list()

mc.speed_move(0.10, 0.10)
for i in range(POINTS_NUMBER):
    data_point = mc.peek_motor_info()
    time.sleep(0.05)
    for key in data_frame:
        data_frame[key].append(data_point[key])

for key in data_frame:
    plt.plot(data_frame[key])
    plt.show()

mc.speed_move(0, 0)
print(data_frame)
print(data_frame.keys())
scipy.io.savemat('pid_data.mat', data_frame)
print('finished')

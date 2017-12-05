#!/usr/bin/env python3
'''This modules graphes and collects antenna data'''

from RobotCommunicator import RobotCommunicator
import matplotlib
matplotlib.use('QT4agg')
import matplotlib.pyplot as plt
import numpy as np
mc = RobotCommunicator()
data_frame = mc._get_antenna_data_points()
for i in range(5):
    print(mc.get_antenna_data())
# plt.plot(data_frame)
# plt.show()

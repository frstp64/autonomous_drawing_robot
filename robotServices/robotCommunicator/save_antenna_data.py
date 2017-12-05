#!/usr/bin/env python3
'''This modules graphes and collects antenna data'''

import pickle
from RobotCommunicator import RobotCommunicator
my_communicator = RobotCommunicator()

data = []
for i in range(100):
    data.append(my_communicator._get_antenna_data_points())
    print("Trame {} terminée!".format(i))

print("Fin!")
pickle.dump(data, open("antenna.p", "wb"))

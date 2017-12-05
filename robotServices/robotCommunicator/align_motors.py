#!/usr/bin/env python3
'''This modules graphes and collects antenna data'''
import time
from RobotCommunicatorDebugging import RobotCommunicatorDebugging

my_communicator = RobotCommunicatorDebugging()

my_communicator.set_control_loop_mode("single_wheel")
time.sleep(0.1)
for i in range(4):
    print("motor {}".format(i))
    my_communicator.set_wheel_command(i, 1)
    time.sleep(1)
    print(my_communicator.peek_motor_info())
    my_communicator.set_wheel_command(i, -1)
    time.sleep(1)
    print(my_communicator.peek_motor_info())
    my_communicator.set_wheel_command(i, 0)
    time.sleep(1)
print("wow")
print(my_communicator.peek_motor_info())

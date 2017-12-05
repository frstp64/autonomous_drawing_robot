#!/usr/bin/env python3
'''This modules graphes and collects antenna data'''

from RobotCommunicator import RobotCommunicator
import matplotlib
matplotlib.use('QT4agg')
mc = RobotCommunicator()
mc.position_move_blocking(0.40, 0)
mc.position_move_blocking(0, 0.1)
mc.position_move_blocking(-0.40, 0)
mc.position_move_blocking(0, 0.1)
mc.position_move_blocking(-0.1, 0)
mc.position_move_blocking(0, -0.3)
mc.position_move_blocking(0.1, 0)
mc.position_move_blocking(0, 0.1)

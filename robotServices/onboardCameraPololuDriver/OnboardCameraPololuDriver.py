#!/usr/bin/env python3
"""This module contains a polulu object used to control the Micro Maestro
Pololu device with 6 servos"""

import time
import threading
import serial
import serial.tools.list_ports

MAXTARGET = 14400
MINTARGET = 0


class OnboardCameraPololuDriver:
    """This module controls one pololu"""
    def __init__(self):
        available_ports = list(serial.tools.list_ports.grep('Pololu'))
        available_ports.sort()
        self.serial_port = serial.Serial(available_ports[0].device)
        self.serial_lock = threading.RLock()

    def __serial_write(self, command):
        self.serial_lock.acquire()
        self.serial_port.write(command)
        time.sleep(1)
        self.serial_lock.release()

    def set_target(self, channel_number, target):
        """Sets the target location for a given channel.

        Inputs:
            channel_number -- The channel number, 0 to 5.
            target -- The target position, a float from 0 to 1.
        """

        target = int(target * (2**14-1))

        command = [0x84, channel_number, target & 0x7F, (target >> 7) & 0x7F]
        self.__serial_write(command)

    def set_speed(self, channel_number, speed):
        """Sets the speed for a given channel.

        Inputs:
            channel_number -- The channel number, 0 to 5.
            speed -- The target speed, a float from 0 to 1.
        """
        speed = int(speed * (2**14-1))

        command = [0x87, channel_number, speed & 0x7F, (speed >> 7) & 0x7F]
        self.__serial_write(command)

    def set_acceleration(self, channel_number, acceleration):
        """Sets the acceleration for a given channel.

        Inputs:
            channel_number -- The channel number, 0 to 5.
            acceleration -- The target acceleration, a float from 0 to 1.
        """

        acceleration = acceleration * (2**14-1)
        command = [0x89, channel_number, acceleration & 0x7F,
                   (acceleration >> 7) & 0x7F]
        self.__serial_write(command)

    def get_position(self, channel_number):
        """Returns the position of the given channel.

        Inputs:
            channel_number -- The number of the channel, 0 to 5.

        Output:
            Returns the position between 0 and 1.
        """
        command = [0x90, channel_number]
        self.__serial_write(command)
        response = self.serial_port.read(2) / (2**14-1)
        return response

    def is_moving(self):
        """Checks if any servo is moving.
        Output:
            Returns a boolean indicating if any servo is moving.
        """
        command = [0xA1]
        self.__serial_write(command)
        response = self.serial_port.read(1)
        return response != b'\x00'

    def get_errors(self):
        """Returns the error value of the Pololu.
        Output:
            Returns the error code.
        """
        command = [0xA1]
        self.__serial_write(command)
        response = self.serial_port.read(2)
        return response

    def go_home(self):
        """Sends all servos to their default position."""
        command = [0xA2]
        self.__serial_write(command)

    def lower_gripper(self):
        """Lowers the gripper.
           Assumes that the gripper is on the fifth channel."""
        self.set_target(5, 0.45)

    def lift_gripper(self):
        """Lifts the gripper.
           Assumes that the gripper is on the fifth channel."""
        self.set_target(5, 0.65)


if __name__ == "__main__":
    myPololu = OnboardCameraPololuDriver()
    myPololu.set_speed(5, 0)
    while True:
        print('boop1')
        myPololu.lower_gripper()
        time.sleep(1)
        print('boop2')
        myPololu.lift_gripper()
        time.sleep(1)

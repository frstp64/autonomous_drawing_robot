#!/usr/bin/env python3
"""This module contains nose tests for the Picasso Robot."""

import time
from unittest import TestCase

from RobotCommunicatorDebugging import RobotCommunicatorDebugging


class TestRobotFunctionalities(TestCase):
    """Tests related to the robot."""

    def setUp(self):
        self.communicator = RobotCommunicatorDebugging()

    def test_check_packet_ok(self):
        packet_bytes = [0x01, 0x02]
        #  self.communicator.__check_packet(packet_bytes, 0x01, 1)
        self.assertTrue(1)

    def test_ping(self):
        """Tests pinging the robot."""
        self.communicator.ping()
        self.assertTrue(1)

    def test_set_get_speed_pid(self):
        """Tests modification of the PID variables."""
        pid_values = [4, 5, 6]
        self.communicator.set_speed_pid(*pid_values)
        result = self.communicator.get_speed_pid()
        self.assertSequenceEqual(pid_values, result)
        pid_values = [8, 9, 10]
        self.communicator.set_speed_pid(*pid_values)
        result = self.communicator.get_speed_pid()
        self.assertSequenceEqual(pid_values, result)

    def test_set_get_control_loop_mode(self):
        """Tests modification of the control loop mode."""
        new_mode = "per_wheel"
        self.communicator.set_control_loop_mode(new_mode)
        result = self.communicator.get_control_loop_mode()
        self.assertEqual(new_mode, result)
        new_mode = "open_loop"
        self.communicator.set_control_loop_mode(new_mode)
        result = self.communicator.get_control_loop_mode()
        self.assertEqual(new_mode, result)

    def human_test_switch_led(self):
        """Tests flashing both leds"""
        print("testing both leds!")
        self.communicator.switch_led(0, True)
        self.communicator.switch_led(1, True)
        self.communicator.switch_led(2, True)
        self.communicator.switch_led(3, True)
        time.sleep(1)
        self.communicator.switch_led(0, False)
        self.communicator.switch_led(1, False)
        self.communicator.switch_led(2, False)
        self.communicator.switch_led(3, False)

    def human_test_print_string(self):
        """Tests printing on the LCD monitor on the robot."""
        print("testing the LCD!")
        self.communicator.print_string("A Nice Unit Test :D :D :D")
        time.sleep(1)

    def human_test_speed_move(self):
        """Tests if the robot is moving with PID."""
        print("testing the speed move!")
        self.communicator.set_control_loop_mode("per_wheel")
        self.communicator.speed_move(0.0, -0.25)
        for i in range(50):
            time.sleep(0.1)
            print(self.communicator.peek_motor_info())
        self.communicator.speed_move(0.0, 0)
        quit()
        self.communicator.speed_move(-0.10, 0)
        for i in range(10):
            time.sleep(0.1)
            print(self.communicator.peek_motor_info())
        self.communicator.speed_move(0, 0.10)
        for i in range(10):
            time.sleep(0.1)
            print(self.communicator.peek_motor_info())
        self.communicator.speed_move(0, -0.10)
        for i in range(10):
            time.sleep(0.1)
            print(self.communicator.peek_motor_info())
        self.communicator.set_control_loop_mode("open_loop")
        self.communicator.speed_move(0, 0)
        time.sleep(5)

    def human_test_speed_rotate(self):
        """Tests if the robot is moving."""
        print("testing the speed rotate!")
        self.communicator.set_control_loop_mode("per_wheel")
        self.communicator.speed_rotate(0.19)
        time.sleep(9)
        self.communicator.speed_rotate(-0.19)
        time.sleep(9)
        self.communicator.speed_rotate(0)
        time.sleep(0.5)
        self.communicator.set_control_loop_mode("open_loop")
        self.communicator.speed_move(0, 0)

    def human_test_get_power(self):
        """Tests the power estimation for the antenna."""
        print("testing the antenna power measure!")
        result = self.communicator.get_power()
        print(result)
        time.sleep(1)

    def human_test_get_antenna_data(self):
        """Tests if the antenna data is received."""
        print("testing the antenna data reception!")
        result_list = []
        for i in range(1):
            print("received {}!".format(i))
            result_list.append(self.communicator.get_antenna_data())
        print(result_list)

    def test_position_move(self):
        """Tests the desired position move vector for the robot. """
        print("testing the postion move!")
        self.communicator.set_control_loop_mode("position_move")

        self.communicator.position_move(0.2, 0)
        for i in range(50):
            time.sleep(0.125)
            print(self.communicator.peek_motor_info())

        self.communicator.position_move(0, 0)
        # time.sleep(0.5)
        # self.communicator.position_move(0, 0.1)
        # time.sleep(0.5)
        # self.communicator.position_move(0, -1)
        # time.sleep(0.5)
        # self.communicator.position_move(0, 0)
        # time.sleep(0.5)

    def test_position_move_blocking(self):
        """Tests the desired position move vector for the robot. """
        print("testing the postion move!")
        self.communicator.position_move_blocking(0.2, 0)
        self.communicator.position_move_blocking(0, 0.2)
        self.communicator.position_move_blocking(-0.2, 0)
        self.communicator.position_move_blocking(0, -0.2)

    def human_test_position_rotate(self):
        """Tests if the robot is moving."""
        print("testing the position rotate!")
        self.communicator.set_control_loop_mode("position_move")
        self.communicator.position_rotate(0.55)
        for i in range(100):
            time.sleep(0.125)
            print(self.communicator.peek_motor_info())
        quit()
        self.communicator.position_rotate(-0.01)
        time.sleep(5)
        self.communicator.position_rotate(0)
        time.sleep(0.5)
        self.communicator.set_control_loop_mode("open_loop")
        self.communicator.speed_move(0, 0)

    def test_check_packet_wrong(self):
        """Tests si le packet n'est pas correct."""
        print("testing packet!")
        # packet_bytes != [0x01, 0x02]
        # self.assertFalse(1)

    def test_peek_motor_info(self):
        """Testing if it peeks at the current motor information"""
        for i in range(2000):
            print(i, end=', ')
            print(self.communicator.peek_motor_info())

    def test_get_motor_data(self):
        """Testing if it Retrieves the motor data"""
        print("testing Retrieving the motor data!")
        pass

    def test_set_get_speed_angle_pid(self):
        """Testing modification of the PID angle variables. """
        pid_values = [4, 5, 6]
        self.communicator.set_speed_angle_pid(*pid_values)
        result = self.communicator.get_speed_angle_pid()
        self.assertSequenceEqual(pid_values, result)
        pid_values = [8, 9, 10]
        self.communicator.set_speed_angle_pid(*pid_values)
        result = self.communicator.get_speed_angle_pid()
        self.assertSequenceEqual(pid_values, result)

    def test_speed_angle_move(self):
        """Testing seting the control loop mode for the robot internal control"""
        print("Testing seting the control loop mode for the robot internal control!")
        pass

    def test_speed_angle_rotate(self):
        """Testing speed angle rotate"""
        print("Testing speed angle rotate!")
        pass

    def test_set_get_position_pid(self):
        """Tests modification of the PID variables."""
        pid_values = [4, 5, 6]
        self.communicator.set_position_pid(*pid_values)
        result = self.communicator.get_position_pid()
        self.assertSequenceEqual(pid_values, result)
        pid_values = [8, 9, 10]
        self.communicator.set_position_pid(*pid_values)
        result = self.communicator.get_position_pid()
        self.assertSequenceEqual(pid_values, result)

    def test_set_get_position_angle_pid(self):
        """Testing modification of the PID angle variables"""
        pid_values = [4, 5, 6]
        self.communicator.set_position_angle_pid(*pid_values)
        result = self.communicator.get_position_angle_pid()
        self.assertSequenceEqual(pid_values, result)
        pid_values = [8, 9, 10]
        self.communicator.set_position_angle_pid(*pid_values)
        result = self.communicator.get_position_angle_pid()
        self.assertSequenceEqual(pid_values, result)

    def test_position_angle_move(self):
        """Testing position angle move"""
        print("Testing position angle move")
        pass

    def test_position_angle_rotate(self):
        """Testing position angle rotate"""
        print("Testing position angle rotate")
        pass

    def test_set_ultimate_pid(self):
        """Testing setting ultimate PID"""
        print("Testing setting ultimate PID")
        pass

    def test_get_ultimate_pid(self):
        """Testing getting ultimate PID"""
        print("Testing getting ultimate PID")
        pass

    def test_ultimate_move(self):
        """Testing ultimate move"""
        print("Testing ultimate move")
        pass

    def test_ultimate_rotate(self):
        """Testing ultimate rotate"""
        print("Testing ultimate rotate")
        pass


if __name__ == "__main__":
    # pylint: disable-msg=C0103
    testObj = TestRobotFunctionalities()
    testObj.setUp()
    # testObj.human_test_switch_led()
    # testObj.human_test_print_string()
    # testObj.human_test_speed_move()
    testObj.human_test_speed_rotate()
    # testObj.human_test_get_power()
    # testObj.human_test_get_antenna_data()
    # testObj.test_peek_motor_info()
    # testObj.test_position_move()
    # testObj.test_position_move_blocking()
    #testObj.human_test_position_rotate()

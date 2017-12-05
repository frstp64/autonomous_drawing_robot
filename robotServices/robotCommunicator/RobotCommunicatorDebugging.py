#!/usr/bin/env python3
"""This module contains an interface to communicate with the onboard
microcontroller (STM32), extra functions are available here for
debugging purposes."""
try:
    from .RobotCommunicator import RobotCommunicator
    from .PacketID import IDDebug
except: 
    from RobotCommunicator import RobotCommunicator
    from PacketID import IDDebug

MODE_DICT = {"open_loop": 0x0,
             "per_wheel": 0x1,
             "position_move": 0x3,
             "single_wheel": 0xFF}
REVERSE_MODE_DICT = {0x0: "open_loop",
                     0x1: "per_wheel",
                     0x3: "position_move",
                     0xFF: "single_wheel"}
BYTES_PER_FRAME = 128
FRAME_NUMBER_MOTOR = 32
CONVERSION_RATIO = 1000000  # we want micrometers per second


class RobotCommunicatorDebugging(RobotCommunicator):
    """This module controls one stm32f407 discovery."""

    def set_speed_pid(self, param_p, param_i, param_d):
        """Sets the PID value for the wheel PID

        Inputs:
            param_p -- The P parameter of the PID, 32 bit integer
            param_i -- The I parameter of the PID, 32 bit integer
            param_d -- The D parameter of the PID, 32 bit integer
        """

        p_bytes = super()._int32_to_bytes(param_p)
        i_bytes = super()._int32_to_bytes(param_i)
        d_bytes = super()._int32_to_bytes(param_d)

        super()._send_packet([IDDebug.set_speed_pid, *p_bytes, *i_bytes,
                              *d_bytes])

    def get_speed_pid(self):
        """Retrieves the currently used per-wheel PID for speed

        Output:
            Returns a tuple containing the PID values
        """
        reception_pkt = super()._send_receive_packet([IDDebug.get_speed_pid],
                                                     4*3)
        param_p = super()._bytes_to_int32(reception_pkt[1:4])
        param_i = super()._bytes_to_int32(reception_pkt[5:9])
        param_d = super()._bytes_to_int32(reception_pkt[9:13])

        return (param_p, param_i, param_d)

    def set_control_loop_mode(self, mode):
        """Sets the control loop mode for the robot internal control

        Inputs:
            mode -- The mode, a string, must be in the following options:
                "open_loop", "per_wheel"

        Note: Other control modes might be added later.
        """
        super()._send_packet([IDDebug.set_cl_mode, MODE_DICT[mode]])

    def get_control_loop_mode(self):
        """Retrieves the current loop mode of the robot

        Output:
            Returns the current mode as a string"""
        reception_packet = super()._send_receive_packet([IDDebug.get_cl_mode],
                                                        1)
        return REVERSE_MODE_DICT[reception_packet[1]]

    def peek_motor_info(self):
        """Peeks at the current motor information.

        Output:
            Returnss a dictionary containing the following information:
            'command': a tuple containing the current command for each motor
            'measure': a tuple containing the current_measure for each motor
            Note: both are in meters per second
        """
        reception_packet = super()._send_receive_packet([IDDebug.peek_motor],
                                                        12*4)
        wheel_0_command = super()._bytes_to_int32(reception_packet[1:5])
        wheel_1_command = super()._bytes_to_int32(reception_packet[5:9])
        wheel_2_command = super()._bytes_to_int32(reception_packet[9:13])
        wheel_3_command = super()._bytes_to_int32(reception_packet[13:17])
        wheel_0_speed = super()._bytes_to_int32(reception_packet[17:21])
        wheel_1_speed = super()._bytes_to_int32(reception_packet[21:25])
        wheel_2_speed = super()._bytes_to_int32(reception_packet[25:29])
        wheel_3_speed = super()._bytes_to_int32(reception_packet[29:33])
        wheel_0_output = super()._bytes_to_int32(reception_packet[33:37])
        wheel_1_output = super()._bytes_to_int32(reception_packet[37:41])
        wheel_2_output = super()._bytes_to_int32(reception_packet[41:45])
        wheel_3_output = super()._bytes_to_int32(reception_packet[45:49])
        return {'command': (wheel_0_command,
                            wheel_1_command,
                            wheel_2_command,
                            wheel_3_command),
                'measure': (wheel_0_speed,
                            wheel_1_speed,
                            wheel_2_speed,
                            wheel_3_speed),
                'output': (wheel_0_output,
                           wheel_1_output,
                           wheel_2_output,
                           wheel_3_output)}

    def start_motor_data_capture(self):
        """Starts motor data capture."""
        super()._send_packet([IDDebug.start_motor_data_capture])

    def _get_motor_data_readiness(self):
        """Checks if motor data is ready."""
        reception_packet = super()._send_receive_packet([IDDebug.get_m_ready],
                                                        1)
        return reception_packet[1] == 1

    def get_motor_data(self):
        """Retrieves the motor data.

        Output: Returns a dictionary containing:
                the motor commands, outputs and encoder values
        Content of the dictionary:
        'command': a list of list, one for each motor
        'output': a list of list, one for each motor
        'encodervalue': a list of list, one for each motor

        Note: MUST CALL start_motor_data_capture before!
        """

        while not self._get_motor_data_readiness():
            pass

        motor_data = []

        for current_frame_number in range(FRAME_NUMBER_MOTOR):
            curr_frame_bytes = super()._uint16_to_bytes(current_frame_number)
            new_data_pkt = super()._send_receive_packet([IDDebug.get_f_motor,
                                                         *curr_frame_bytes],
                                                        BYTES_PER_FRAME)
            motor_data.extend(new_data_pkt[1:])

        motor_points = []
        for i in range(len(motor_data)//2):
            motor_points.append(int.from_bytes(motor_data[2*i:2*i+2],
                                               'little'))
        # The data points are arranged in the following fashion:
        # 4xcommand, 4xoutput, 4xencoder
        # here we untangle the data points
        motor_data_dict = dict()
        motor_data_dict['command'] = [motor_points[0:len(motor_points):12],
                                      motor_points[1:len(motor_points):12],
                                      motor_points[2:len(motor_points):12],
                                      motor_points[3:len(motor_points):12]]

        motor_data_dict['output'] = [motor_points[4:len(motor_points):12],
                                     motor_points[5:len(motor_points):12],
                                     motor_points[6:len(motor_points):12],
                                     motor_points[7:len(motor_points):12]]

        motor_data_dict['encoder'] = [motor_points[8:len(motor_points):12],
                                      motor_points[9:len(motor_points):12],
                                      motor_points[10:len(motor_points):12],
                                      motor_points[11:len(motor_points):12]]

        return motor_data_dict

    def set_position_pid(self, param_p, param_i, param_d):
        """Sets the PID value for the position PID.

        Inputs:
            param_p -- The P parameter of the PID, 32 bit integer.
            param_i -- The I parameter of the PID, 32 bit integer.
            param_d -- The D parameter of the PID, 32 bit integer.
        """

        p_bytes = super()._int32_to_bytes(param_p)
        i_bytes = super()._int32_to_bytes(param_i)
        d_bytes = super()._int32_to_bytes(param_d)

        super()._send_packet([IDDebug.set_pos_pid, *p_bytes, *i_bytes,
                              *d_bytes])

    def get_position_pid(self):
        """Retrieves the currently used PID for position.

        Output:
            Returns a tuple containing the PID values.
        """
        reception_packet = super()._send_receive_packet([IDDebug.get_pos_pid],
                                                        4*3)
        param_p = super()._bytes_to_int32(reception_packet[1:4])
        param_i = super()._bytes_to_int32(reception_packet[5:9])
        param_d = super()._bytes_to_int32(reception_packet[9:13])

        return (param_p, param_i, param_d)

    def set_position_angle_pid(self, param_p, param_i, param_d):
        """Sets the PID value for the position angle PID.

        Inputs:
            param_p -- The P parameter of the PID, 32 bit integer.
            param_i -- The I parameter of the PID, 32 bit integer.
            param_d -- The D parameter of the PID, 32 bit integer.
        """

        p_bytes = super()._int32_to_bytes(param_p)
        i_bytes = super()._int32_to_bytes(param_i)
        d_bytes = super()._int32_to_bytes(param_d)

        super()._send_packet([IDDebug.set_pos_ang_pid, *p_bytes, *i_bytes,
                              *d_bytes])

    def get_position_angle_pid(self):
        """Retrieves the currently used PID for speed angle.

        Output:
            Returns a tuple containing the PID values.
        """
        reception_pkt = super()._send_receive_packet([IDDebug.get_p_ang_pid],
                                                     4*3)
        param_p = super()._bytes_to_int32(reception_pkt[1:4])
        param_i = super()._bytes_to_int32(reception_pkt[5:9])
        param_d = super()._bytes_to_int32(reception_pkt[9:13])

        return (param_p, param_i, param_d)

    def set_speed_angle_pid(self, param_p, param_i, param_d):
        """Sets the PID value for the speed angle PID.

        Inputs:
            param_p -- The P parameter of the PID, 32 bit integer.
            param_i -- The I parameter of the PID, 32 bit integer.
            param_d -- The D parameter of the PID, 32 bit integer.
        """

        p_bytes = super()._int32_to_bytes(param_p)
        i_bytes = super()._int32_to_bytes(param_i)
        d_bytes = super()._int32_to_bytes(param_d)

        super()._send_packet([IDDebug.set_speed_ang_pid, *p_bytes, *i_bytes,
                              *d_bytes])

    def get_speed_angle_pid(self):
        """Retrieves the currently used PID for position angle.

        Output:
            Returns a tuple containing the PID values.
        """
        reception_pkt = super()._send_receive_packet([IDDebug.get_s_ang_pid],
                                                     4*3)
        param_p = super()._bytes_to_int32(reception_pkt[1:4])
        param_i = super()._bytes_to_int32(reception_pkt[5:9])
        param_d = super()._bytes_to_int32(reception_pkt[9:13])

        return (param_p, param_i, param_d)

    def set_wheel_command(self, wheel_number, command):
        """Sets a command for a single wheel.

        Inputs:
            wheel_number -- The number of the wheel, from 0 to 3.
            command -- a command to send to the wheel, a float from 0 to 1.
        """
        command_value = int(command * CONVERSION_RATIO)
        cmd_bytes = super()._int32_to_bytes(command_value)
        super()._send_packet([IDDebug.set_wheel_cmd, wheel_number, *cmd_bytes])

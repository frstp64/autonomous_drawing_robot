#!/usr/bin/env python3
"""This module contains an interface to communicate with the onboard
microcontroller (STM32), only the functions useful for control are
available here."""

import time
try:
    from .RobotCommunicatorEssentials import RobotCommunicatorEssentials
    # from decodeFred import decode_frame
    from .manchester_decoder import Manchester
    from .PacketID import PacketID
    from .PacketID import IDDebug
except:
    from RobotCommunicatorEssentials import RobotCommunicatorEssentials
    # from decodeFred import decode_frame
    from manchester_decoder import Manchester
    from PacketID import PacketID
    from PacketID import IDDebug

BYTES_PER_FRAME = 128
FRAME_NUMBER = 16
CONVERSION_RATIO = 1000000  # we want micrometers per second

MODE_DICT = {"open_loop": 0x0,
             "per_wheel": 0x1,
             "position_move": 0x3,
             "single_wheel": 0xFF}

REVERSE_MODE_DICT = {0x0: "open_loop",
                     0x1: "per_wheel",
                     0x3: "position_move",
                     0xFF: "single_wheel"}


class RobotCommunicator(RobotCommunicatorEssentials):
    """This module controls one stm32f407 discovery."""

    def ping(self):
        """Pings the microcontroller.

        Output:
            Returns True if ping succeeded, False otherwise
        """
        super()._send_receive_packet([PacketID.ping, 0x01], 1)
        return True

    def switch_led(self, led_number, new_state):
        """Changes a robot LED state.

        Inputs:
            led_number -- The number of the led to change, integer from 0 to 4
            new_state -- The state of the LED, boolean

            Note: 0 for blue, 1 is for orange, 2 is for red, 3 is for green
        """
        state_byte = 0x01 if new_state else 0x00
        super()._send_packet([PacketID.switch_led, led_number, state_byte])

    def print_string(self, text):
        """Writes text on the embedded LCD display.

        Inputs:
            text -- The text, an ASCII string, must be 16 in length
        """
        text = text.encode('ascii', 'ignore')
        text = text.decode()
        text = text[:32]

        text = "{:32}".format(text)
        text = text.encode('ascii', 'ignore')

        super()._send_packet([PacketID.print_string, *text])

    def speed_move(self, speed_x, speed_y):
        """Sets the desired speed for the robot.

        Inputs:
            speed_x -- The x speed as a float in meters per second.
            speed_y -- The y speed as a float in meters per second.

        Note: speed move and speed rotation are mutually exclusive
        in the robot.
        """

        speed_x_integer = int(min(speed_x * CONVERSION_RATIO, 1000000))
        speed_y_integer = int(min(speed_y * CONVERSION_RATIO, 1000000))
        speed_x_bytes = super()._int32_to_bytes(speed_x_integer)
        speed_y_bytes = super()._int32_to_bytes(speed_y_integer)

        super()._send_packet([PacketID.speed_move,
                               *speed_x_bytes,
                               *speed_y_bytes])

    def speed_rotate(self, speed_radians):
        """Sets the desired rotation speed for the robot.

        Inputs:
            speed_radians -- The rotation speed as a float in radians per
            second.
        Note: speed move and speed rotation are mutually exclusive in
        the robot.
        """
        speed_radians_integer = int(speed_radians*CONVERSION_RATIO,
                                        )
        speed_radians_bytes = super()._int32_to_bytes(speed_radians_integer)
        super()._send_packet([PacketID.speed_rotate, *speed_radians_bytes])

    def get_power(self):
        """Retrieves the antenna power value.

        Output:
            Returns the power value in volts as a float.
        """
        reception_packet = super()._send_receive_packet([PacketID.get_power],
                                                        2)
        power_int = super()._bytes_to_uint16(reception_packet[1:])
        power_value = 3.3 * power_int / (2**12-1)
        return power_value

    def _start_data_capture(self):
        """Starts data capture for the antenna."""
        super()._send_packet([PacketID.start_data_capture])

    def _get_data_readiness(self):
        """Checks if antenna data is ready."""
        try:
            reception_packet = super()._send_receive_packet([PacketID.get_ready],
                                                            1)
        except Exception:
            super().__init__()
            return False
        return reception_packet[1] == 1

    def _get_antenna_data_points(self):
        """Retrieves the antenna data points.

        Output: A list of data points.
        """
        my_control_loop_mode = self.get_control_loop_mode()
        self.set_control_loop_mode("open_loop")
        time.sleep(0.1)

        wrong = True
        while wrong is True:
            wrong = False
            self._start_data_capture()
            while not self._get_data_readiness():
                pass

            antenna_data = []

            for current_frame_number in range(FRAME_NUMBER):
                cur_frm_bytes = super()._uint16_to_bytes(current_frame_number)
                while True:
                    try:
                        new_data_packet = super()._send_receive_packet([PacketID.get_frm,
                                                                        *cur_frm_bytes],
                                                                       BYTES_PER_FRAME)
                        break
                    except Exception:
                        wrong = True
                        break  # Failed, please retake
                        pass
                antenna_data.extend(new_data_packet[1:])

        antenna_data_points = []
        for i in range(len(antenna_data)//2):
            antenna_data_points.append(int.from_bytes(antenna_data[2*i:2*i+2],
                                                      'little'))
        # antenna_code = Manchester.DecodeManchester(antenna_data_points)
        # antenna_code = decode_frame(antenna_data_points)
        self.set_control_loop_mode(my_control_loop_mode)
        return antenna_data_points

    def get_antenna_data(self):
        """Retrieves the antenna data.

        Output: A tuple containing (imageNumber (0 to 7),
                                    orientation (0 to 3),
                                    factor(2 or 4))
        """

        antenna_data_points = self._get_antenna_data_points()
        try:
            antenna_code = Manchester.DecodeManchester(antenna_data_points)
        except Exception:
            antenna_code = None
        if antenna_code is None:
            return antenna_code

        antenna_code_tuple = (antenna_code[0]*2**2 + antenna_code[1]*2 + antenna_code[2],
                              (antenna_code[3]*2 + antenna_code[4]),
                              (antenna_code[5]+1)*2)
        return antenna_code_tuple

    def position_move(self, position_x, position_y):
        """Sets the desired position move vector for the robot.

        Inputs:
            speed_x -- The x displacement as a float in meters.
            speed_y -- The y displacement as a float in meters.

        Note: position move and position rotate are mutually exclusive
        in the robot.

        Note: THIS IS RELATIVE TO THE ROBOT'S POSITION
              WHEN IT RECEIVES THE COMMAND.
        """

        position_x_integer = int(position_x * CONVERSION_RATIO)
        position_y_integer = int(position_y * CONVERSION_RATIO)
        position_x_bytes = super()._int32_to_bytes(position_x_integer)
        position_y_bytes = super()._int32_to_bytes(position_y_integer)

        super()._send_packet([PacketID.position_move,
                              *position_x_bytes,
                              *position_y_bytes])

    def position_rotate(self, position_radians):
        """Sets the desired rotation angle for the robot.

        Inputs:
            position_radians -- The rotation speed as a float in radians per
            second.
        Note: position move and position rotate are mutually exclusive in
        the robot.

        Note: THIS IS RELATIVE TO THE ROBOT'S ANGLE
              WHEN IT RECEIVES THE COMMAND.
        """
        pos_rads_int = int(position_radians*CONVERSION_RATIO)
        pos_rads_bytes = super()._int32_to_bytes(pos_rads_int)
        super()._send_packet([PacketID.position_rotate, *pos_rads_bytes])

    def _is_position_finished(self):
        """Checks if the position move or rotate is finished
        Returns a boolean.
        """
        reception_packet = super()._send_receive_packet([PacketID.check_position_over], 1)
        return reception_packet[1] == 1

    def position_move_blocking(self, position_x, position_y):
        """Sets the desired position move vector for the robot.

        Inputs:
            speed_x -- The x displacement as a float in meters.
            speed_y -- The y displacement as a float in meters.

        Note: position move and position rotate are mutually exclusive
        in the robot.

        Note: THIS IS RELATIVE TO THE ROBOT'S POSITION
              WHEN IT RECEIVES THE COMMAND.
        """
        self.position_move(position_x, position_y)
        is_finished = False
        while is_finished is False:
            time.sleep(0.1)
            is_finished = self._is_position_finished()
        time.sleep(0.2)

    def position_rotate_blocking(self, position_radians):
        """Sets the desired rotation angle for the robot.

        Inputs:
            position_radians -- The rotation speed as a float in radians per
            second.
        Note: position move and position rotate are mutually exclusive in
        the robot.

        Note: THIS IS RELATIVE TO THE ROBOT'S ANGLE
              WHEN IT RECEIVES THE COMMAND.
        """
        self.position_rotate(position_radians)
        is_finished = False
        while is_finished is False:
            time.sleep(0.1)
            is_finished = self._is_position_finished()

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


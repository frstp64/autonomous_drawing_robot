#!/usr/bin/env python3
"""This module contains the packetID definitions."""

from enum import IntEnum


class PacketID(IntEnum):
    """The packet IDs for each type of normal command
    """
    speed_rotate = 0x00
    speed_move = 0x01
    ping = 0x03
    switch_led = 0x05
    print_string = 0x06
    get_power = 0x0D
    get_ready = 0x0F
    start_data_capture = 0x11
    get_frm = 0x12
    position_rotate = 0x1B
    position_move = 0x1C
    check_position_over = 0x27


class IDDebug(IntEnum):
    """The packet IDs for each type of debugging command
    """
    set_speed_pid = 0x07
    get_speed_pid = 0x08
    set_cl_mode = 0x0A
    get_cl_mode = 0x0B
    peek_motor = 0x14
    start_motor_data_capture = 0x16
    get_m_ready = 0x17
    get_f_motor = 0x19
    set_pos_pid = 0x1D
    get_pos_pid = 0x1E
    set_pos_ang_pid = 0x20
    get_p_ang_pid = 0x21
    set_speed_ang_pid = 0x23
    get_s_ang_pid = 0x24
    set_wheel_cmd = 0x26

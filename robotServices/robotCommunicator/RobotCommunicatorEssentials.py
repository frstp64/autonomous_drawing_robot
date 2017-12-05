#!/usr/bin/env python3
"""This module contains an interface to communicate with the onboard
microcontroller (STM32), the essential modules contains the
functions for barebones communication, it is not very useful
for real world testing."""

import os
import threading
import serial
import serial.tools.list_ports
from cobs import cobs


class WrongPacketException(Exception):
    """Custom Exception for packets
    Inputs:
        bytes_list: a list of bytes
    """
    def __init__(self, bytes_list):
        byte_str = ":".join("{:02x}".format(c) for c in bytes_list)
        message = "Wrong packet received! The bytes: {}".format(byte_str)
        super(WrongPacketException, self).__init__(message)


# pylint: disable-msg=R0903
class RobotCommunicatorEssentials(object):
    """This module controls one stm32f407 discovery."""
    def __init__(self):
        self.serial_port = None
        if os.name == 'nt':
            available_ports = list(
                serial.tools.list_ports.grep('STM'))
            available_ports.sort()
            for a_port in available_ports:
                try:
                    self.serial_port = serial.Serial(a_port[0], timeout=1)
                except Exception:
                    pass
        else:
            available_ports = list(
                serial.tools.list_ports.grep('STM32 Virtual ComPort'))
            available_ports.sort()
            self.serial_port = serial.Serial(available_ports[0].device, timeout=1)
        self.serial_lock = threading.RLock()
        if self.serial_port is None:
            raise Exception("aucun port fonctionnel")

    def _send_packet(self, packet):
        my_packet = bytearray(cobs.encode(bytearray(packet)))
        my_packet.append(0x00)
        self.serial_lock.acquire()
        self.serial_port.write(my_packet)
        self.serial_lock.release()

    def _send_receive_packet(self, packet, payload_size):
        wanted_id = packet[0]+1
        self.serial_lock.acquire()
        self._send_packet(packet)
        packet_to_return = None
        my_packet = bytearray()
        while len(my_packet) != payload_size+3:
            my_temp_packet = bytearray(self.serial_port.read(payload_size+3))
            # print(my_temp_packet)
            my_packet = my_temp_packet
            # print("woow")
            if len(my_packet) != (payload_size+3):
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                print(len(my_temp_packet))
                print(payload_size + 3)
                self.serial_port.read(payload_size+3)
                self._send_packet(packet)
        self.serial_lock.release()
        my_packet.pop()  # Removal of the zero byte at the end
        packet_to_return = cobs.decode(my_packet)
        try:
            self._check_packet(packet_to_return, wanted_id, payload_size)
        except Exception as e:
            print(e)
            self.serial_port.close()
            self.__init__()
        return packet_to_return

    @classmethod
    def _check_packet(cls, packet_bytes, wanted_id, payload_size):
        if (len(packet_bytes) != payload_size+1 or
                packet_bytes[0] != wanted_id):
            raise WrongPacketException(packet_bytes)

    @classmethod
    def _int32_to_bytes(cls, intp):
        bytes_list = list(intp.to_bytes(4, byteorder='little', signed=True))
        return bytes_list

    @classmethod
    def _uint16_to_bytes(cls, intp):
        bytes_list = list(intp.to_bytes(2, byteorder='little', signed=False))
        return bytes_list

    @classmethod
    def _bytes_to_uint16(cls, bytes_list):
        return int.from_bytes(bytes_list, 'little', signed=False)

    @classmethod
    def _bytes_to_int32(cls, bytes_list):
        return int.from_bytes(bytes_list, 'little', signed=True)

#!/usr/bin/env python

import socket
import struct

# from pyjoystick.sdl2 import Key, Joystick, run_event_loop


# def print_add(joy):
#     print('Added', joy)


# def print_remove(joy):
#     print('Removed', joy)


# def key_received(key):
#     print(key, key.value)
#     buf = struct.pack('>10sd', bytes(str(key), 'ascii'), key.value)
#     sock.sendto(buf, (UDP_IP, UDP_PORT))

import pywinusb.hid as hid
import time

# USB Hardware ID
target_vendor_id = 0x0A9F
target_product_id = 0x0009

# AXIS ID
X = 0
Y = 1
Z = 2
Rz = 3
Sl0 = 4
Sl1 = 5


class Joystick:
    def __init__(self, device_index=0, socket_callback=None):
        self.device_index = device_index
        self.socket_callback = socket_callback
        self.__state_connected = False
        self.axis_value = [0, 0, 0, 0, 0, 0]
        self.inputs_state = [0, 0, 0, 0, 0, 0, 0, 0]
        self.outputs_state = [0, 0, 0, 0]
        hid_devices = hid.HidDeviceFilter(
            VendorId=target_vendor_id, product_id=target_product_id
        )
        all_joystick = hid_devices.get_devices()
        if len(all_joystick) >= 1:  # assumed there is only one joystick
            self.device = all_joystick[device_index]
            self.device.open()
            print("Joystick found: %s!" % repr(self.device))
            self.device.set_raw_data_handler(
                self.raw_inputs_handler
            )  # setup handler to poll joystick status
            self.out_report = (
                self.device.find_output_reports()
            )  # create an output_report object
            self.__state_connected = True
        else:
            raise IOError("No joystick attached to USB")

    def close(self):
        """close connection by calling this method"""
        if self.__state_connected:
            self.clear_outputs()
            self.device.close()  # close the connection and it supposed to kill the thread
            del self  # delete the object - otherwise the main process will still run in windows after the main application is closed
            self.__state_connected = False

    @property
    def connected(self):
        return self.__state_connected

    def show_hids(self):
        """print a report with all HID devices connected"""
        hid.core.show_hids()

    def convert(self, lsb, msb):
        """convert little endian decimal bytes into signed integer"""
        return (msb << 8) + lsb

    def raw_inputs_handler(self, data):
        """process raw hid inputs data : XYZ axis and inputs"""
        # data is a Python list of 9 integers in little endian format
        print("Raw data: {0}".format(data))
        # print(f"{data[5]:08b}:{data[6]:08b}")

        # AXIS
        self.axis_value[X] = self.convert(data[1], data[2])
        self.axis_value[Y] = self.convert(data[3], data[4])
        self.axis_value[Z] = self.convert(data[5], data[6])
        self.axis_value[Rz] = self.convert(data[7], data[8])
        self.axis_value[Sl0] = self.convert(data[9], data[10])
        self.axis_value[Sl1] = self.convert(data[11], data[12])

        # INPUTS
        # convert int value to binary list
        self.inputs_state = [int(x) for x in bin(data[13])[2:].zfill(8)]

        if self.socket_callback is not None:
            self.socket_callback(self.device_index, self.axis_value + self.inputs_state)

    def get_axis(self, index):
        """return the value of the axis 0..2 / return signed integer"""
        return self.axis_value[index]

    def get_input(self, index):
        """return the state of the input 0..15 / return 0 or 1"""
        return self.inputs_state[index]


class Joystick3:
    def __init__(self, device_index=0):
        self.__state_connected = False
        self.axis_value = [0, 0, 0, 0, 0, 0]
        self.inputs_state = [0, 0, 0, 0, 0, 0, 0, 0]
        self.outputs_state = [0, 0, 0, 0]
        hid_devices = hid.HidDeviceFilter(
            VendorId=target_vendor_id, product_id=target_product_id
        )
        all_joystick = hid_devices.get_devices()
        if len(all_joystick) >= 1:  # assumed there is only one joystick
            self.device = all_joystick[device_index]
            self.device.open()
            print("Joystick found: %s!" % repr(self.device))
            self.device.set_raw_data_handler(
                self.raw_inputs_handler
            )  # setup handler to poll joystick status
            self.out_report = (
                self.device.find_output_reports()
            )  # create an output_report object
            self.__state_connected = True
        else:
            raise IOError("No joystick attached to USB")

    def close(self):
        """close connection by calling this method"""
        if self.__state_connected:
            self.clear_outputs()
            self.device.close()  # close the connection and it supposed to kill the thread
            del self  # delete the object - otherwise the main process will still run in windows after the main application is closed
            self.__state_connected = False

    @property
    def connected(self):
        return self.__state_connected

    def show_hids(self):
        """print a report with all HID devices connected"""
        hid.core.show_hids()

    def convert(self, lsb, msb):
        """convert little endian decimal bytes into signed integer"""
        return (msb << 8) + lsb

    def raw_inputs_handler(self, data):
        """process raw hid inputs data : XYZ axis and inputs"""
        # data is a Python list of 9 integers in little endian format
        print("Raw data: {0}".format(data))
        # print(f"{data[5]:08b}:{data[6]:08b}")

        # AXIS
        self.axis_value[X] = self.convert(data[1], data[2])
        self.axis_value[Y] = self.convert(data[3], data[4])
        self.axis_value[Z] = self.convert(data[5], data[6])
        self.axis_value[Rz] = self.convert(data[7], data[8])
        self.axis_value[Sl0] = self.convert(data[9], data[10])
        # self.axis_value[Sl1] = self.convert(data[11], data[12])

        if self.socket_callback is not None:
            self.socket_callback(self.axis_value)

    def get_axis(self, index):
        """return the value of the axis 0..2 / return signed integer"""
        return self.axis_value[index]

    def get_input(self, index):
        """return the state of the input 0..15 / return 0 or 1"""
        return self.inputs_state[index]


if __name__ == "__main__":
    # Main configuration
    UDP_IP = "127.0.0.1"  # Vehicle IP address
    UDP_PORT = 24421  # This port match the ones using on other scripts

    update_rate = 0.1  # 10 hz loop cycle
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def socket_callback(joy_id, msg):
        payload = struct.pack(">15d", joy_id, msg)
        sock.sendto(payload, (UDP_IP, UDP_PORT))

    devices = hid.HidDeviceFilter(
        VendorId=target_vendor_id, product_id=target_product_id
    ).get_devices()

    print(" i  par       ser  ven  prd  ver  name")
    for i, dev in enumerate(devices):
        print(
            "{0:> 3} {1:> 2} {2:>9} {3:04X} {4:04X} {5:04X}  {6:}".format(
                i,
                dev.parent_instance_id,
                dev.serial_number,
                dev.vendor_id,
                dev.product_id,
                dev.version_number,
                dev.product_name,
            )
        )

    joy_1 = Joystick(3, socket_callback=socket_callback)
    # joy_2 = Joystick(2)
    # joy_3 = Joystick(3)
    joys = [joy_1]  # , joy_2, joy_3]
    joy_1.show_hids()
    # my_joystick.set_output(2, FASTBLINK)
    print("Starting...")
    while True:
        # 10Hz sample rate
        time.sleep(1 / 10.0)


# run_event_loop(print_add, print_remove, key_received)

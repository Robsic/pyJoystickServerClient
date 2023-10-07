# import pywinusb.hid as hid

# from time import sleep
# from msvcrt import kbhit


# def sample_handler(data):
#     print("Raw data: {0:}".format(data))


# selection = int(input("\nEnter number to select device (0-{})\n".format(i)))
# device = devices[selection]
# print("You have selected {}".format(device.product_name))
# try:
#     device.open()
#     # set custom raw data handler
#     device.set_raw_data_handler(sample_handler)

#     print("\nWaiting for data...\nPress any (system keyboard) key to stop...")
#     while not kbhit() and device.is_plugged():
#         # just keep the device opened to receive events
#         sleep(0.5)
# finally:
#     device.close()
# Python Class to handle an USB joystick - HID compliant
# Reference hardware is APEM USB joystick interface based on JoyWarrior chip
# The interface can handle 3x 10 bits axis, 16 inputs, 4 outputs
# Require Pywinusb : https://github.com/rene-aguirre/pywinusb
#
# Adapted to Python 3+ from Python 2.7
# DPo 2/2020

import pywinusb.hid as hid


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


class Joystick:
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
        # self.axis_value[X] = self.convert(data[1], data[2])
        # self.axis_value[Y] = self.convert(data[3], data[4])
        # self.axis_value[Z] = self.convert(data[5], data[6])
        # self.axis_value[Rz] = self.convert(data[7], data[8])
        # self.axis_value[Sl0] = self.convert(data[9], data[10])
        # self.axis_value[Sl1] = self.convert(data[11], data[12])

        # INPUTS
        # convert int value to binary list
        # self.inputs_state = [int(x) for x in bin(data[13])[2:].zfill(8)]
        # print("State: ", self.inputs_state)
        # print("Axes: ")
        # for ax in self.axis_value:
        #     print(ax, end=" ")
        # print()
        # print self.inputs_state

    def get_axis(self, index):
        """return the value of the axis 0..2 / return signed integer"""
        return self.axis_value[index]

    def get_input(self, index):
        """return the state of the input 0..15 / return 0 or 1"""
        return self.inputs_state[index]

    def set_output(self, index, state):
        """control the state of an output 0..3 / state 0 or 1"""
        self.outputs_state[index] = state
        # buffer=[0x0,0x0,0x0,0x0,0x0]
        buffer = [0] + self.outputs_state  # first item of the list is ID=0x00
        self.out_report[0].set_raw_data(buffer)
        self.out_report[0].send()

    def clear_outputs(self):
        """clear all outputs"""
        self.outputs_state = [0, 0, 0, 0]
        buffer = [0] + self.outputs_state  # first item of the list is ID=0x00
        self.out_report[0].set_raw_data(buffer)
        self.out_report[0].send()


import time


if __name__ == "__main__":
    joy_1 = Joystick(3)
    # joy_2 = Joystick(2)
    # joy_3 = Joystick(3)
    joys = [joy_1]  # , joy_2, joy_3]
    joy_1.show_hids()
    # my_joystick.set_output(2, FASTBLINK)
    while True:
        # for i, joy in enumerate(joys):
        #     print("JS", i, " Axis: ", joy.axis_value)
        #     print("JS", i, " Buttons: ", joy.inputs_state)

        time.sleep(1 / 10.0)
    # my_joystick.set_output(0, ON)
    # my_joystick.clear_outputs()

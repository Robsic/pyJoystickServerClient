#!/usr/bin/env python

import socket
import struct
from pyjoystick.sdl2 import Key, Joystick, run_event_loop


def print_add(joy):
    print('Added', joy)


def print_remove(joy):
    print('Removed', joy)


def key_received(key):
    print(key, key.value)
    buf = struct.pack('>10sd', bytes(str(key), 'ascii'), key.value)
    sock.sendto(buf, (UDP_IP, UDP_PORT))


# Main configuration
UDP_IP = "127.0.0.1"  # Vehicle IP address
UDP_PORT = 24421  # This port match the ones using on other scripts

update_rate = 0.1  # 10 hz loop cycle
# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


run_event_loop(print_add, print_remove, key_received)

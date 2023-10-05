#!/usr/bin/env python

import socket
import struct
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Publisher(Node):

    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(String, 'joystick', 10)

    def publish(self, msg):
        self.publisher_.publish(msg)


def main(args=None):
    # Main configuration
    UDP_IP = "127.0.0.1"  # Vehicle IP address
    UDP_PORT = 24421  # This port match the ones using on other scripts
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    rclpy.init(args=args)
    publisher = Publisher()

    while True:
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        id, val = struct.unpack('>10sd', data)
        print("received message: ", id.decode('ascii'), val)
        msg = String()
        msg.data = str(id.decode('ascii')) + ":" + str(val)
        publisher.publish(msg)
        rclpy.spin_once(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

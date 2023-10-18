#!/usr/bin/env python

import socket
import struct

import rclpy

from rclpy.node import Node

from std_msgs.msg import String, Int8
from sirv_msgs.msg import CAT793FPanelControl, CAT793FVehicleControl

def normalize(x, min, max, min_n, max_n):
    return (max_n-min_n)/(max-min)*(x-max)+max_n

def getHoistPosition(value):
    if value > 145 and value <= 155:
        return int(ord('D'))
    elif value > 445 and value <= 455:
        return int(ord('F'))
    elif value > 1020 and value <= 1030:
        return int(ord('S'))
    elif value > 1355 and value <= 1365:
        return int(ord('U'))
    else:
        return int(ord('F'))
    
def getTransmissionPosition(value):
    if value > 140 and value <= 150:
        return int(ord('P'))
    elif value > 325 and value <= 335:
        return int(ord('R'))
    elif value > 675 and value <= 685:
        return int(ord('N'))
    elif value > 1020 and value <= 1030:
        return int(ord('D'))
    elif value > 1250 and value <= 1260:
        return 2
    elif value > 1335 and value <= 1345:
        return 1
    else:
        return int(ord('N'))

class Publisher(Node):
    def __init__(self):
        super().__init__("joystick_publisher")
        self.vehicle_control_publisher = self.create_publisher(CAT793FVehicleControl, "vehicle_control", 10)
        self.panel_control_publisher = self.create_publisher(CAT793FPanelControl, "panel_control", 10)

    def publish_vc(self, msg):
        self.vehicle_control_publisher.publish(msg)

    def publish_pc(self, msg):
        self.panel_control_publisher.publish(msg)


def main(args=None):
    # Main configuration
    UDP_IP = "172.16.10.145" #"127.0.0.1"  # Server IP address (local pc)
    UDP_PORT = 24421  # This port match the ones using on other scripts
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    rclpy.init(args=args)
    publisher = Publisher()
    print("Listening...")
    try:
        msg_vc = CAT793FVehicleControl()
        while True:
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
            val = struct.unpack(">15i", data)
            
            
            if val[0] == 1:
                msg_vc.brake = normalize(val[5], 259, 1210, 0, 1)
                msg_vc.throttle = normalize(val[6], 263, 1250, 0, 1)
                msg_vc.brakeretarder = normalize(val[4], 142, 1390, 0, 1)
                msg_vc.hoist = getHoistPosition(val[3])

            if val[0] == 2:
                msg_vc.emergencybrake = normalize(val[6], 263, 1212, 0, 1)
                msg_vc.transmission = getTransmissionPosition(val[3])

            if val[0] == 3:
                print("raw: ", val)
                msg_vc.steeringwheel = normalize(val[1], -1000, 1000, -1, 1)


            msg_vc.header.stamp = publisher.get_clock().now().to_msg()
            publisher.publish_vc(msg_vc)

            # msg = String()
            # msg.data = str(val)
            # publisher.publish(msg)

    except(KeyboardInterrupt):
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

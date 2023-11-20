#!/usr/bin/env python

import socket
import struct

import rclpy

from rclpy.node import Node

from std_msgs.msg import String, Int8
from sirv_msgs.msg import CAT793FPanelControl, CAT793FVehicleControl

currentGear = 'N'

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
    global currentGear
    if value > 135 and value <= 155:
        currentGear = int(ord('P'))
    elif value > 320 and value <= 340:
        currentGear = int(ord('R'))
    elif value > 670 and value <= 690:
        currentGear = int(ord('N'))
    elif value > 1015 and value <= 1035:
        currentGear = int(ord('D'))
    elif value > 1245 and value <= 1265:
        currentGear = int(ord('2'))
    elif value > 1330 and value <= 1350:
        currentGear = int(ord('1'))
    else:
        return currentGear
    return currentGear

class Publisher(Node):
    def __init__(self):
        super().__init__("joystick_publisher")
        self.vehicle_control_publisher = self.create_publisher(CAT793FVehicleControl, "cat793f/vehicle_control", 10)
        self.panel_control_publisher = self.create_publisher(CAT793FPanelControl, "cat793f/panel_control", 10)

    def publish_vc(self, msg):
        self.vehicle_control_publisher.publish(msg)

    def publish_pc(self, msg):
        self.panel_control_publisher.publish(msg)

OFF = 0
ON = 1
CRANK = 2

def main(args=None):
    # Main configuration
    UDP_IP = "172.16.10.145" #"127.0.0.1"  # Server IP address (local pc)
    UDP_PORT = 24421  # This port match the ones using on other scripts
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    rclpy.init(args=args)
    publisher = Publisher()
    print("Listening...")

    key_state = OFF

    try:
        msg_vc = CAT793FVehicleControl()
        msg_pc = CAT793FPanelControl()
        while True:
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
            val = struct.unpack(">15i", data)
            
            if val[0] == 1:
                print("1: ", val)
                msg_vc.brake = normalize(val[5], 259, 1210, 0, 1)
                msg_vc.throttle = normalize(val[6], 263, 1250, 0, 1)
                msg_vc.brakeretarder = 1 - normalize(val[4], 142, 1390, 0, 1)
                msg_vc.hoist = getHoistPosition(val[3])
                if(val[11] == 1):
                    msg_pc.horn = True
                else:
                    msg_pc.horn = False

                if(key_state == ON and val[13] == 1):
                    key_state == CRANK
                    msg_pc.enginekeyon = True
                    msg_pc.enginekeycrank = True
                elif(key_state == CRANK and val[13] == 0):
                    key_state == ON
                    msg_pc.enginekeyon = True
                    msg_pc.enginekeycrank = False


            if val[0] == 2:
                # print("2: ", val)
                msg_vc.emergencybrake = normalize(val[6], 263, 1212, 0, 1)
                msg_vc.transmission = getTransmissionPosition(val[3])

                if(key_state == OFF and val[11] == 1):
                    key_state = ON
                    msg_pc.enginekeyon = True
                    msg_pc.enginekeycrank = False
                elif(key_state == ON and val[11] == 0):
                    key_state = OFF
                    msg_pc.enginekeyon = False
                    msg_pc.enginekeycrank = False

            if val[0] == 3:
                msg_vc.steeringwheel = normalize(val[1], -1000, 1000, -1, 1)

            time_now = publisher.get_clock().now().to_msg()
            msg_vc.header.stamp = time_now
            msg_pc.header.stamp = time_now
            publisher.publish_vc(msg_vc)
            publisher.publish_pc(msg_pc)

    except(KeyboardInterrupt):
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python

import rclpy       # the imports match the dependencies from package. xml
from rclpy.node import Node     # import python node class
from std_msgs.msg import String # import the built-in string message type
import serial
from time import sleep

SER_PORT = "/dev/ttyUSB0"
SER_SPEED = 9600

ser = serial.Serial(SER_PORT, SER_SPEED) # open the serial port
for i in range(10000):
    sleep(0.1)
    data = [50,60,70,80,90]
    data2 = bytearray(data)
    ser.write(data2)
    sleep(0.1)
ser.close()

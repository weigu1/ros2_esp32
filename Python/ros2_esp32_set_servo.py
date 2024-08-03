#!/usr/bin/env python

import rclpy       # the imports match the dependencies from package. xml
from rclpy.node import Node     # import python node class
from std_msgs.msg import String # import the built-in string message type
import serial
from time import sleep

SER_PORT = "/dev/ttyUSB0"
SER_SPEED = 1000000

""" I could not import an external module, so I added the usb_serial code here"""
class SerialComm():
    """ Get a value over Serial from ESP32, Frame = Preamble (2 byte) + data
        (2 byte for now) + CRC (1byte) """
    def __init__(self, port, speed):
        self.port = port
        self.speed = speed
        self.preambel = b'\xAB\xCD' # define the sync word as a bytes object
        self.data_index1 = 2 # first byte index of data values in data_stream (for CRC)
        self.data_index2 = 3 # last byte index of data values in data stream (for CRC)
        self.pot_val_index1 = 2 # hbyte index of pot value in data_stream
        self.pot_val_index2 = 3 # lbyte index of pot value in data_stream
        self.servo_val_index1 = 2 # hbyte index of servo value in data_stream
        self.servo_val_index2 = 3 # lbyte index of servo value in data_stream
        self.crc_index = 4   # crc index in data stream (1 byte)
        self.data_bytes_pot = 5
        self.data_bytes_servo = 5

    def check_if_crc_ok(self, crc, crc_data):
        """ Compare data crc with with checksum """
        checksum = self.calculate_checksum(crc_data)
        for byte in crc_data:   # iterate over each byte in the data
            checksum ^= byte    # XOR the byte with the checksum
        if crc == checksum:
            return True
        else:
            return False

    def calculate_checksum(self, crc_data):
        """ xor checksum over bytes from index1 to index2 included"""
        checksum = 0
        for byte in crc_data:   # iterate over each byte in the data
            checksum ^= byte    # XOR the byte with the checksum
        return checksum

    def get_pot_value(self):
        ser = serial.Serial(self.port, self.speed) # open the serial port
        data = ser.read(self.data_bytes_pot)
        #print(data.hex())
        if data[0:2]  == self.preambel:
            pot_value = data[self.pot_val_index1]*256+data[self.pot_val_index2]
            #print("Pot value: ", pot_value, end="\t")
            crc = data[self.crc_index]
            crc_data = data[self.data_index1:self.data_index2+1]    # checksum data values
            if self.check_if_crc_ok(crc, crc_data):
                #print("Checksum ok")
                flag = True
            else:
                #print("Checksum error")
                flag = False
        else:
            #print("Data acquisition error!")
            flag = False
        ser.close()
        if flag:
            return pot_value
        else:
            return -1

    def send_angle_2_servo(self, angle):
        ser = serial.Serial(self.port, self.speed) # open the serial port
        data = [None]*5
        data[0:2] = self.preambel
        data[self.servo_val_index1:self.servo_val_index2+1] = angle.to_bytes(2, 'big')
        data[self.crc_index] = self.calculate_checksum(data[2:4])
        ser.write(bytearray(data))
        ser.close()

    def scan_open_serial_ports(self):
        """ Scans for available serial ports that are not blocked by other software"""
        portnames = []
        for i in range(20):  # Linux ttyS,  AMA, ACM, USB
            try:
                name = "/dev/ttyUSB"+str(i)
                s = serial.Serial(name)
                s.close()
                portnames.append (name)
                name = "/dev/ttyACM"+str(i)
                s = serial.Serial(name)
                s.close()
                portnames.append (name)
                name = "/dev/ttyAMA"+str(i)
                s = serial.Serial(name)
                s.close()
                portnames.append (name)
                name = "/dev/ttyS"+str(i)
                s = serial.Serial(name)
                s.close()
                portnames.append (name)
            except:
                pass
        return portnames

    def print_open_serial_ports(self):
        """ Prints available serial ports that are not blocked by other software"""
        print ("\nSerial ports that are not in use: ")
        portnames = self.scan_open_serial_ports()
        for port in portnames:
            print (port, end="\t")
        print()

class Send2Servo(Node): # another pub node class
    def __init__(self):
        self.angle = 0
        super().__init__('send_angle_2_servo')
        self.subscription = self.create_subscription(
            String,
            'pot_value',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5    # 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.serial_comm = SerialComm(SER_PORT, SER_SPEED)

    def timer_callback(self): # is called every 0.5s from timer
        self.serial_comm.send_angle_2_servo(self.angle)
        self.get_logger().info('I send: "%s"' % str(self.angle))

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.angle = int(int(msg.data)*180/4095)

def main(args=None):
    rclpy.init(args=args)               # init rclpy
    send_angle_to_servo = Send2Servo()  # create publisher node
    rclpy.spin(send_angle_to_servo)    # loop the node so callbacks occur
    send_angle_to_servo.destroy_node()  # destroy the node explicitly
    rclpy.shutdown()                    # shutdown rclpy

if __name__ == '__main__':
    main()

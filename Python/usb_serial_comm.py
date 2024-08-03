#!/usr/bin/env python
import serial

SER_PORT = "/dev/ttyUSB0"
SER_SPEED = 115200

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
        self.crc_index = 4   # crc index in data stream (1 byte)

    def check_if_crc_ok(self, crc, crc_data):
        """ xor checksum over bytes from index1 to index2 included
            and compare with data checksum """
        checksum = 0
        for byte in crc_data:   # iterate over each byte in the data
            checksum ^= byte    # XOR the byte with the checksum
        if crc == checksum:
            return True
        else:
            return False

    def get_pot_value(self):
        ser = serial.Serial(self.port, self.speed) # open the serial port
        data = ser.read(5)
        print(data.hex())
        if data[0:2]  == self.preambel:
            pot_value = data[self.pot_val_index1]*256+data[self.pot_val_index2]
            print("Pot value: ", pot_value, end="\t")
            crc = data[self.crc_index]
            crc_data = data[self.data_index1:self.data_index2+1]    # checksum data values
            if self.check_if_crc_ok(crc, crc_data):
                print("Checksum ok")
                flag = True
            else:
                print("Checksum error")
                flag = False
        else:
            print("Data acquisition error!")
            flag = False
        ser.close() 
        if flag:
            return pot_value
        else:
            return -1

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

def main(args=None):
    sc = SerialComm(SER_PORT, SER_SPEED)
    sc.print_open_serial_ports()
    sc.get_pot_value()

if __name__ == '__main__':
    main()

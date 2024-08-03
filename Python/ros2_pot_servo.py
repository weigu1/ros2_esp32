# Scan serial ports
import serial

PREAMBEL = b'\xAB\xCD' # define the sync word as a bytes object
data_index1 = 2 # first byte index of data values in data_stream (for CRC)
data_index2 = 3 # last byte index of data values in data stream (for CRC)
pot_val_index1 = 2 # hbyte index of pot value in data_stream
pot_val_index2 = 3 # lbyte index of pot value in data_stream

crc_index = 4   # crc index in data stream (1 byte)

def scan_open_serial_ports():
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

def print_open_serial_ports():
    """ Prints available serial ports that are not blocked by other software"""
    print ("\nSerial ports that are not in use: ")
    portnames = scan_open_serial_ports()
    for port in portnames:
        print (port, end="\t")
    print()

def check_if_crc_ok(crc, crc_data):
    """ xor checksum over bytes from index1 to index2 included
        and compare with data checksum """
    checksum = 0
    for byte in crc_data:   # iterate over each byte in the data
        checksum ^= byte    # XOR the byte with the checksum
    if crc == checksum:
        return True
    else:
        return False

def get_pot_value():
    ser = serial.Serial('/dev/ttyUSB0', 115200) # open the serial port
    data = ser.read(5)
    if data[0:2]  == PREAMBEL:
        pot_value = data[pot_val_index1]*256+data[pot_val_index2]
        print("Pot value: ", pot_value, end="\t")
        crc = data[crc_index]
        crc_data = data[data_index1:data_index2+1]    # checksum data values
        if check_if_crc_ok(crc, crc_data):
            print("Checksum ok")
            flag = True
        else:
            print("Checksum error")
            flag = False
    else:
        print("Data acquisition error!")
        flag = False
    ser.close() 
    return flag


if __name__ == "__main__":
    print_open_serial_ports()
    get_pot_value()
    get_pot_value()


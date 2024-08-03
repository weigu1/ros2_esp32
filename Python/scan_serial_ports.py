# Scan serial ports
import glob
import serial
def Scan_serial_dev():
    """ Scans /dev folder for serial ports """
    return glob.glob('/dev/tty[A-Za-z]*') # search for device files that match the pattern for serial ports

def Scan_open_serial():
    """ Scans for available serial ports that are not blocked by other software"""
    Portnames = []
    for i in range(20):  # Linux ttyS,  AMA, ACM, USB
        try:
            name = "/dev/ttyUSB"+str(i)
            s = serial.Serial(name)
            s.close()
            Portnames.append (name)
            name = "/dev/ttyACM"+str(i)
            s = serial.Serial(name)
            s.close()
            Portnames.append (name)
            name = "/dev/ttyAMA"+str(i)
            s = serial.Serial(name)
            s.close()
            Portnames.append (name)
            name = "/dev/ttyS"+str(i)
            s = serial.Serial(name)
            s.close()
            Portnames.append (name)
        except:
            pass
    return Portnames

if __name__ == "__main__":
    print ("Serial ports in /dev folder: ")
    serial_dev_ports = Scan_serial_dev()
    for port in serial_dev_ports:
        print(port, end="\t") # print the name of each serial port
    print ("\nSerial ports that are not in use: ")
    portnames = Scan_open_serial()
    for port in portnames:
        print (port)

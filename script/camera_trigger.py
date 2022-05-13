import serial
import sys 
import time 

class Camera:
    def __init__(self):
        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        self.ser.bytesize=serial.EIGHTBITS
        self.ser.parity=serial.PARITY_NONE
        self.ser.stopbits=serial.STOPBITS_ONE
        self.ser.port="/dev/ttyUSB1"
        self.ser.open()

    def trigger(self):
        self.ser.write('t'.encode('utf-8'))
        time.sleep(1)

    def repeat(self):
        self.ser.write('r'.encode('utf-8'))
    
    def stop(self):
        self.ser.write('s'.encode('utf-8'))

    def close(self):
        try:
            self.ser.close()
            print ("Closed serial port.")
        except:
            sys.exit ("Couldn't close serial port.")

if __name__=='__main__':
    camera = Camera()
    camera.trigger()
    camera.close()


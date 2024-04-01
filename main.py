import serial
import time

teensy = serial.Serial("/dev/tty.usbmodem87906701")
teensy.baudrate = 9600
time.sleep(5)
teensy.write(1)
teensy.close()
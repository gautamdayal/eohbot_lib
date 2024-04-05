import serial
import time
import perception

teensy = serial.Serial("/dev/tty.usbmodem87906701")
teensy.baudrate = 9600

def encode_angle(angle):
    return str(angle).zfill(3)

while True:
    teensy.write(encode_angle(0).encode())
    time.sleep(5)
    teensy.write(encode_angle(999).encode())
    time.sleep(10)
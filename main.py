import serial
import time
import perception

teensy = serial.Serial("/dev/tty.usbmodem87906701")
teensy.baudrate = 9600

def encode_angle(angle):
    return str(angle).zfill(3)

while True:
    teensy.write(encode_angle(0).encode())
    time.sleep(3)
    teensy.write(encode_angle(90).encode())
    time.sleep(3)
    teensy.write(encode_angle(180).encode())
    time.sleep(3)
    teensy.write(encode_angle(270).encode())
    time.sleep(3)

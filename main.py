import serial
import time
import perception
import numpy as np

teensy = serial.Serial("/dev/ttyACM0")
teensy.baudrate = 9600

def waypoint_following(waypoints):
    angle_delay_pairs = []
    for i in range(len(waypoints) - 1):
        curr_wp = waypoints[i]
        next_wp = waypoints[i+1]
        dist = 250 * np.linalg.norm(np.array(curr_wp) - np.array(next_wp))
        curr_delay = dist/406
        curr_angle = np.rad2deg(np.arctan2(next_wp[0]-curr_wp[0], next_wp[1] - curr_wp[1])) % 360
        angle_delay_pairs.append((int(curr_angle), curr_delay))
    return angle_delay_pairs
        

def encode_angle(angle):
    return str(angle).zfill(3)

sample_waypoints = [(0, 0), (1, 0), (1, 1), (2, 2)]
motor_commands = waypoint_following(sample_waypoints)
for cmd in motor_commands:
    teensy.write(encode_angle(cmd[0]).encode())
    time.sleep(cmd[1])
    teensy.write(encode_angle(999).encode())
    time.sleep(0.5)



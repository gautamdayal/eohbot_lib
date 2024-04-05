import serial
import time
import perception
import numpy as np
import cv2
import matplotlib.pyplot as ppl
import pupil_apriltags as apriltag
from perception.Perception import *
from planning.astar import *

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

## EOH CONFIG
dyn_ids = [29,32,30,31,34]
static_ids = [13,21,2,4,16,11,20,17,10,14,15,12,22,19,24,9,23,18]
objectPoints = {
    13: np.array([[616+tagsize, 805+tagsize, 0.], [616., 805+tagsize, 0.], [616., 805., 0.], [616+tagsize, 805, 0.]]),
    21: np.array([[1226+tagsize, 805+tagsize, 0.], [1226., 805+tagsize, 0.], [1226., 805., 0.], [1226+tagsize, 805, 0.]]),
    2: np.array([[1836., 805+tagsize, 0.], [1836., 805., 0.], [1836+tagsize, 805., 0.], [1836+tagsize, 805+tagsize, 0.]]),
    4: np.array([[2446+tagsize, 805+tagsize, 0.], [2446., 805+tagsize, 0.], [2446., 805., 0.], [2446+tagsize, 805., 0.]]),
    16: np.array([[2446., 1414., 0.], [2446+tagsize, 1414., 0.], [2446+tagsize, 1414+tagsize, 0.], [2446., 1414+tagsize, 0.]]),
    11: np.array([[1836., 1414., 0.], [1836+tagsize, 1414., 0.], [1836+tagsize, 1414+tagsize, 0.], [1836., 1414+tagsize, 0.]]),
    20: np.array([[1226+tagsize, 1414+tagsize, 0.], [1226., 1414+tagsize, 0.], [1226., 1414., 0.], [1226+tagsize, 1414., 0.]]),
    17: np.array([[616+tagsize, 1414., 0.], [616+tagsize, 1414+tagsize, 0.], [616., 1414+tagsize, 0.], [616., 1414., 0.]]),
    10: np.array([[2446., 2020., 0.], [2446+tagsize, 2020., 0.], [2446+tagsize, 2020+tagsize, 0.], [2446., 2020+tagsize, 0.]]),
    14: np.array([[1836., 2020+tagsize, 0.], [1836., 2020., 0.], [1836+tagsize, 2020., 0.], [1836+tagsize, 2020+tagsize, 0.]]),
    15: np.array([[1226., 2020., 0.], [1226+tagsize, 2020., 0.], [1226+tagsize, 2020+tagsize, 0.], [1226., 2020+tagsize, 0.]]),
    12: np.array([[616., 2020+tagsize, 0.], [616., 2020., 0.], [616+tagsize, 2020., 0.], [616+tagsize, 2020+tagsize, 0.]]),
    22: np.array([[616+tagsize, 2437., 22.], [616+tagsize, 2437, 22+tagsize], [616, 2437, 22+tagsize], [616., 2437., 22.]]),
    19: np.array([[1226+tagsize, 2437, 55+tagsize], [1226, 2437, 55+tagsize],[1226., 2437., 55.], [1226+tagsize, 2437., 55.]]),
    24: np.array([[1836, 2437, 22+tagsize], [1836., 2437., 22.], [1836+tagsize, 2437., 22.], [1836+tagsize, 2437, 22+tagsize]]),
    9: np.array([[2446+tagsize, 2437, 23+tagsize], [2446, 2437, 23+tagsize], [2446., 2437., 23.], [2446+tagsize, 2437., 23.]]),
    23: np.array([[3050., 1414., 32.], [3050, 1414., 32 + tagsize], [3050, 1414+tagsize, 32 + tagsize], [3050., 1414+tagsize, 32.]]),
    18: np.array([[3050, 805+tagsize, 16 + tagsize], [3050., 805+tagsize, 16.], [3050., 805., 16.], [3050, 805., 16 + tagsize]])
}

vs = cv2.VideoCapture(0)
fig = ppl.figure(figsize=(4,4))
axes = ppl.axes(projection='3d')
axes.set_box_aspect([1.0,1.0,1.0])
axes.set_xlabel('X (mm)')
axes.set_ylabel('Y (mm)')
axes.set_zlabel('Z (mm)')
axes.azim = -90
axes.elev = 40

for _,objectPoint in objectPoints.items():
    axes.scatter3D(objectPoint[0, 0], objectPoint[0, 1], objectPoint[0, 2], '-k', c='blue')

    axes.plot3D((objectPoint[0, 0], objectPoint[1, 0]), (objectPoint[0, 1], objectPoint[1, 1]),
                (objectPoint[0, 2], objectPoint[1, 2]), '-g')
    axes.plot3D((objectPoint[1, 0], objectPoint[2, 0]), (objectPoint[1, 1], objectPoint[2, 1]),
                (objectPoint[1, 2], objectPoint[2, 2]), '-g')
    axes.plot3D((objectPoint[2, 0], objectPoint[3, 0]), (objectPoint[2, 1], objectPoint[3, 1]),
                (objectPoint[2, 2], objectPoint[3, 2]), '-g')
    axes.plot3D((objectPoint[3, 0], objectPoint[0, 0]), (objectPoint[3, 1], objectPoint[0, 1]),
                (objectPoint[3, 2], objectPoint[0, 2]), '-g')
ppl.show()

while vs.isOpened():
    ret, frame = vs.read()
    if not ret:
        print("Can't get frame")
        break
    
    camera, obstacle_points = get_robot_obstacle_points(frame, axes)
    occupancy, start = generate_occupancy(camera, obstacle_points)
    occupancy_graph = get_graph(occupancy)
    waypoints = get_waypoints(get_path(occupancy_graph, (1, 1), (12, 10)))
    motor_commands = waypoint_following(waypoints)
    for cmd in motor_commands:
        teensy.write(encode_angle(cmd[0]).encode())
        time.sleep(cmd[1])
        teensy.write(encode_angle(999).encode())
        time.sleep(0.5)
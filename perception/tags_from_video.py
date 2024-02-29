import matplotlib.pyplot as plt 
from pupil_apriltags import Detection, Detector
import numpy as np 
import cv2

camera_matrix = np.array([[1.95512684e+03, 0.00000000e+00, 9.99979035e+02],
       [0.00000000e+00, 1.96894288e+03, 7.03714795e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

at_detector = Detector(
   families="tag25h9",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Unable to open camera")
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't get frame")
        break
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    output = at_detector.detect(img_gray,
    estimate_tag_pose=True,
    camera_params=[camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]],
    tag_size = 0.1)
    for det in output:
        curr_center=det.center 
        cv2.putText(img_gray, f"{det.tag_id}", (int(curr_center[0]), int(curr_center[1])),
            cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4, cv2.LINE_AA)

    cv2.imshow("frame", img_gray)
    if cv2.waitKey(1) == ord('q'):
        break 

cap.release()
cv2.destroyAllWindows()
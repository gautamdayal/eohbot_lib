import cv2
import numpy as np
import matplotlib.pyplot as ppl
import pupil_apriltags as apriltag


def PolyArea2D(pts):
    l = np.hstack([pts, np.roll(pts, -1, axis=0)])
    a = 0.5 * abs(sum(x1 * y2 - x2 * y1 for x1, y1, x2, y2 in l))
    return a


def plotCamera3D(Cesc, rvec, ax=None):
    if ax is None:
        ax = ppl.axes(projection='3d')
    point = ax.scatter3D(Cesc[0], Cesc[1], Cesc[2], 'k', c='red')
    R, _ = cv2.Rodrigues(rvec)

    p1_cam = [-50, 50, 20]
    p2_cam = [50, 50, 20]
    p3_cam = [50, -50, 20]
    p4_cam = [-50, -50, 20]
    
    x_cam = [300,0,0]
    y_cam = [0,300,0]
    z_cam = [0,0,300]

    p1_esc = R.T @ p1_cam + Cesc
    p2_esc = R.T @ p2_cam + Cesc
    p3_esc = R.T @ p3_cam + Cesc
    p4_esc = R.T @ p4_cam + Cesc
    
    x_esc = R.T @ x_cam + Cesc
    y_esc = R.T @ y_cam + Cesc
    z_esc = R.T @ z_cam + Cesc
    
    camera_plot = [ax.plot3D((Cesc[0], p1_esc[0]), (Cesc[1], p1_esc[1]), (Cesc[2], p1_esc[2]), '-k'),
                   ax.plot3D((Cesc[0], p2_esc[0]), (Cesc[1], p2_esc[1]), (Cesc[2], p2_esc[2]), '-k'),
                   ax.plot3D((Cesc[0], p3_esc[0]), (Cesc[1], p3_esc[1]), (Cesc[2], p3_esc[2]), '-k'),
                   ax.plot3D((Cesc[0], p4_esc[0]), (Cesc[1], p4_esc[1]), (Cesc[2], p4_esc[2]), '-k'),
                   ax.plot3D((p1_esc[0], p2_esc[0]), (p1_esc[1], p2_esc[1]), (p1_esc[2], p2_esc[2]), '-k'),
                   ax.plot3D((p2_esc[0], p3_esc[0]), (p2_esc[1], p3_esc[1]), (p2_esc[2], p3_esc[2]), '-k'),
                   ax.plot3D((p3_esc[0], p4_esc[0]), (p3_esc[1], p4_esc[1]), (p3_esc[2], p4_esc[2]), '-k'),
                   ax.plot3D((p4_esc[0], p1_esc[0]), (p4_esc[1], p1_esc[1]), (p4_esc[2], p1_esc[2]), '-k'),
                   ax.plot3D((Cesc[0], x_esc[0]), (Cesc[1], x_esc[1]), (Cesc[2], x_esc[2]), '-r'),
                     ax.plot3D((Cesc[0], y_esc[0]), (Cesc[1], y_esc[1]), (Cesc[2], y_esc[2]), '-g'),
                        ax.plot3D((Cesc[0], z_esc[0]), (Cesc[1], z_esc[1]), (Cesc[2], z_esc[2]), '-b')]

    return camera_plot, point

def normalize_angle(angle):
    angle = angle % (2 * np.pi)
    if angle > np.pi:
        angle -= 2 * np.pi
    return angle

def getCamera3D(rvec, tvec):
    # Centro óptico de la cámara como un punto 3D expresado en el sistema de la escena
    # t = -R @ Cesc => Cesc = -R^-1 @ t, pero R^-1 = R.T => Cesc = -R.T @ t
    R, _ = cv2.Rodrigues(rvec)
    Cesc = (-R.T @ tvec).reshape(3)

    return Cesc


npz_file = "calibration.npz"
tagsize = 100.0
family = "tag25h9"
camera = 0

## Wall configuration
# dyn_ids = [9,4]
# static_ids = [3,1,7,5]
# ids = {3:0, 1:1, 7:2, 5:3}
# objectPoints = [np.array([[0., 0., 0.], [0., tagsize, 0.], [0., tagsize, tagsize], [0., 0., tagsize]]),
#                 np.array([[0., 201., tagsize], [0., 201., 0.], [0., 201 + tagsize, 0.], [0., 201 + tagsize, tagsize]]),
#                 np.array([[0., 0., -108.], [0., 0., -108 - tagsize,], [0., tagsize, -108 - tagsize], [0., tagsize, -108.]]),
#                 np.array([[-13., -42., tagsize], [-13-tagsize, -42., tagsize], [-13-tagsize, -42., 0.], [-13., -42., 0.]])]

## Living room configuration
dyn_ids = [32]
static_ids = [10,12,14,15,16,17,20]
objectPoints = {
    10: np.array([[0., 0., 160.], [-tagsize, 0., 160.], [-tagsize, 0., 160 + tagsize], [0., 0., 160 + tagsize]]),
    16: np.array([[-335-tagsize, 0., 160 + tagsize], [-335., 0., 160 + tagsize], [-335., 0., 160.], [-335-tagsize, 0., 160.]]),
    17: np.array([[-750., 0., 160 + tagsize], [-750., 0., 160.], [-750-tagsize, 0., 160.], [-750-tagsize, 0., 160 + tagsize]]),
    14: np.array([[-1220.0, 150 + tagsize, 160 + tagsize], [-1220., 150., 160 + tagsize], [-1220., 150., 160.], [-1220., 150 + tagsize, 160.]]),
    15: np.array([[-1220., 810 + tagsize, 160 + tagsize], [-1220., 810., 160 + tagsize], [-1220., 810., 160.], [-1220., 810+tagsize, 160.]]),
    12: np.array([[-315., 425+tagsize, 0.], [-315-tagsize, 425+tagsize, 0.], [-315-tagsize, 425., 0.], [-315., 425., 0.]]),
    20: np.array([[-750., 425., 0.], [-750., 425+tagsize, 0.], [-750-tagsize, 425+tagsize, 0.], [-750-tagsize, 425., 0.]])
}


with np.load(npz_file) as data:
    intrinsics = data['intrinsics']
    dist_coeffs = data['dist_coeffs']



vs = cv2.VideoCapture(camera)
detector = apriltag.Detector(families=family)
fig = ppl.figure(figsize=ppl.figaspect(0.5))
axes = fig.add_subplot(1,2,1,projection='3d')
stats = fig.add_subplot(1,2,2)
axes.set_xlabel('X (mm)')
axes.set_ylabel('Y (mm)')
axes.set_zlabel('Z (mm)')
axes.azim = 30
axes.elev = 20

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

camera_points = []
dyn_tag_points = []
arr_pose_t = []
arr_angles = []
while vs.isOpened():
    lines = []
    ret, image = vs.read()
    if not ret:
        break

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray, 
                              estimate_tag_pose=True,
                              tag_size=tagsize/1000,
                              camera_params=[intrinsics[0, 0], intrinsics[1, 1], intrinsics[0, 2], intrinsics[1, 2]])
    coord_fusion = []
    angle_fusion = []
    areas = []
    dyn_points = []
    
    for r in results:
        
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        imagePoints = r.corners

        ptA, ptB, ptC, ptD = imagePoints
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        

        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)

        # draw the left-down (x, y)-coordinates of the AprilTag
        cv2.circle(image, ptA, 5, (255, 0, 0), -1)

        # draw the tag id on the image
        tagid = "tag_id = " + str(r.tag_id)
        cv2.putText(image, tagid, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if r.tag_id not in static_ids and r.tag_id not in dyn_ids:
            # print("Tag not found", r.tag_id)
            continue
        elif r.tag_id in dyn_ids:
            #x, z useful, y not useful
            print("Dynamic Tag found", r.tag_id)
            # print("Tag info",r.pose_R)
            pose = r.pose_t.reshape(3)
            # pose[2] *= -1
            temp_arr = 1000 * np.array(arr_pose_t)
            
            # stats.cla()
            # stats.plot(temp_arr[:,0], c = "red")
            # stats.plot(temp_arr[:,1], c = "green")
            # stats.plot(temp_arr[:,2], c = "blue")
            
            # camera_points.append(axes.scatter3D(1000 * r.pose_t[0,0], 1000 * r.pose_t[1,0], 1000 * r.pose_t[2,0], 'k', c='purple'))
            dyn_points.append(1000 * pose)
            continue
        
        areas.append((PolyArea2D(imagePoints)))
        _, rotation, translation = cv2.solvePnP(objectPoints[r.tag_id], imagePoints, intrinsics, dist_coeffs)

        camera = getCamera3D(rotation, translation)
        coord_fusion.append(camera)
        angle_fusion.append(rotation)

    if len(areas) > 0:
        total_weight = np.sum(areas)
        ratio = []
        for area in areas:
            ratio.append(area / total_weight)
        ratio = np.array(ratio)
        coord_fusion = np.array(coord_fusion)
        angle_fusion = np.array(angle_fusion)
        camera = np.array([0, 0, 0])
        sin_angle = np.array([0, 0, 0])
        cos_angle = np.array([0, 0, 0])
        master_angle = np.array([0, 0, 0])
        for i in range(len(ratio)):
            camera = camera + (coord_fusion[i] * ratio[i])
            sin_angle = sin_angle + (np.sin(angle_fusion[i]).reshape(3) * ratio[i])
            cos_angle = cos_angle + (np.cos(angle_fusion[i]).reshape(3) * ratio[i])
            master_angle = master_angle + (angle_fusion[i].reshape(3) * ratio[i])
            
        angle = np.arctan(sin_angle / cos_angle)
        master_angle = np.array([normalize_angle(master_angle[0]), normalize_angle(master_angle[1]), normalize_angle(master_angle[2])])

        angle[0] = angle[0] - np.pi
        arr_angles.append(master_angle)
        temp_arr = np.array(arr_angles)
        
        stats.cla()
        stats.plot(temp_arr[:,0], c = "purple")
        stats.plot(temp_arr[:,1], c = "orange")
        stats.plot(temp_arr[:,2], c = "black")
    
        lines, camera_point = plotCamera3D(camera, master_angle, axes)
        camera_points.append(camera_point)
        
        for dyn_point in dyn_points:
            # camera_hoomg = np.array([*camera, 1])
            # dyn_coord = np.dot(dyn_point, camera_hoomg)
            # dyn_coord = dyn_coord[:3] / dyn_coord[3]
            R = cv2.Rodrigues(angle)[0]
            dyn_coord = -R.T @ dyn_point + camera
            # dyn_coord = [camera[0] + dyn_point[0], camera[1] + dyn_point[2], camera[2] + dyn_point[1]]
            dyn_tag_points.append(axes.scatter3D(dyn_coord[0], dyn_coord[1], dyn_coord[2], 'k', c='purple'))

    ppl.pause(0.0000000001)

    for line in lines:
        # axes.lines.remove(line)
        line[0].remove()
    lines.clear()

    if len(dyn_tag_points) > 5:
        dyn_tag_points[0].remove()
        dyn_tag_points = dyn_tag_points[1:]

    if len(camera_points) > 15:
        camera_points[0].remove()
        camera_points = camera_points[1:]
        
    if len(arr_pose_t) > 100:
        arr_pose_t = arr_pose_t[1:]
        
    if len(arr_angles) > 100:
        arr_angles = arr_angles[1:]

    cv2.imshow("camera", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ppl.show()
vs.release()
cv2.destroyAllWindows()
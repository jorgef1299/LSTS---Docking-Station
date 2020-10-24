import numpy as np
import cv2
import yaml

# termination criteria - 2nd argument = square size in mm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 26, 0.001)

objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

source = cv2.VideoCapture(0)

width = 640
height = 480

source.set(cv2.CAP_PROP_FRAME_WIDTH, width)
source.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

found = 0
while found < 10:
    ret, img = source.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
    
    if ret == True:
        objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)
        
        img = cv2.drawChessboardCorners(img, (9,6), corners2, ret)
        
        found += 1
        cv2.imshow('img', img)
        cv2.waitKey(500)
    else:
        print('not found')

cv2.destroyAllWindows()

# calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width,height), 1, (width,height)) #alpha=1

source.release()

# transform the matrix and distortion coefficients to writable lists
data = {'optimal_camera_matrix': np.asarray(newcameramtx).tolist(),
        'dist_coeff': np.asarray(dist).tolist(),
        'roi' : np.asarray(roi).tolist()}

with open("calibration_matrix.yaml", "w") as f:
    yaml.dump(data, f)
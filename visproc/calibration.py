#https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html 
#https://github.com/opencv/opencv
#https://github.com/cyrilli/pose-estimation_python-opencv/blob/master/camcalib.py

import cv2
import numpy as np

CHESSBOARD_SIZE = (9, 6) 
SQUARE_SIZE = 0.025 


object_points_template = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
object_points_template[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
object_points_template *= SQUARE_SIZE


object_points = [] 
image_points = []

cap = cv2.VideoCapture(1)

print("Press 's' to save an image for calibration or 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        refined_corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, refined_corners, ret)

        cv2.putText(frame, "Chessboard Found! Press 's' to save this image.", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow('Calibration', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and ret:
        object_points.append(object_points_template)
        image_points.append(refined_corners)
        print(f"Image saved! Total images: {len(object_points)}")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    object_points, image_points, gray.shape[::-1], None, None
)

print("Camera Matrix:")
print(camera_matrix)
print("Distortion Coefficients:")
print(dist_coeffs)

np.savez("calibration_data_live.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

mean_error = 0
for i in range(len(object_points)):
    img_points2, _ = cv2.projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(image_points[i], img_points2, cv2.NORM_L2) / len(img_points2)
    mean_error += error

print(f"Total reprojection error: {mean_error / len(object_points)}")

import cv2
import numpy as np
import time
from aruco_library import ArucoSingleTracker, load_calibration_data

# Load calibration
calib_path = 'calibration_logi.npz'
camera_matrix, dist_coeffs = load_calibration_data(calib_path)

# Set up ArUco detector
aruco_tracker = ArucoSingleTracker(
    id_to_find=1,
    marker_size=0.254,  # adjust if needed (meters)
    show_video=True,
    camera_matrix=camera_matrix,
    camera_distortion=dist_coeffs
)

print("Place the marker in FRONT of the drone (in its +X direction).")
print("Make sure the drone is level and the camera is pointing straight down.")
print("Hit ESC to quit.\n")

while True:
    found, x, y, z = aruco_tracker.track(loop=False)

    if found:
        print(f"MARKER FOUND at X={x:.2f} cm, Y={y:.2f} cm")

        if abs(x) > abs(y):
            if x > 0:
                print("👉 Marker is to the RIGHT in the image")
            else:
                print("👈 Marker is to the LEFT in the image")
        else:
            if y > 0:
                print("👇 Marker is at the BOTTOM of the image")
            else:
                print("☝️ Marker is at the TOP of the image")

        print("---")
    else:
        print("Looking for marker...")

    key = cv2.waitKey(500)
    if key == 27:  # ESC to quit
        break

cv2.destroyAllWindows()

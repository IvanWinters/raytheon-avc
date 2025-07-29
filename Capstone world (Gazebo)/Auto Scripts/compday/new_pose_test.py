import cv2
import numpy as np
import math
import time
import threading

from new_aruco_lib import ArucoSingleTracker, load_calibration_data

# --- Helper Functions (from new_c1-pose.py) ---

def get_location_meters(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # meters
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return newlat, newlon

def camera_to_uav(x_cam, y_cam):
    # Converts camera frame coordinates to UAV frame.
    x_uav = -y_cam
    y_uav = x_cam
    return x_uav, y_uav

def uav_to_ne(x_uav, y_uav, yaw_rad):
    # Converts UAV frame (x,y) offsets into North-East offsets based on UAV yaw.
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east  = x_uav * s + y_uav * c
    return north, east

def marker_to_ned(gamma, R_cam, t_world):
    """
    Convert a marker's pose into NED offsets.
    
    Parameters:
      gamma : UAV yaw (in radians)
      R_cam : Rotation matrix obtained from pose estimation (camera frame)
      t_world : Translation vector (computed, e.g. by -R_cam.T @ tvec)
      
    Returns:
      Tuple with (north_offset, east_offset)
    """
    # Rotation from UAV body frame to world (NED) frame using UAV yaw.
    R_fb = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0,                0,               1]
    ])
    
    # Rotation from camera frame to UAV body frame.
    R_bc = np.array([
        [0, -1, 0],
        [1,  0, 0],
        [0,  0, 1]
    ])
    
    # Overall transformation from camera frame to world (NED) frame.
    R_fa = R_fb @ R_bc @ R_cam
    NED = R_fa @ t_world
    return NED[0], NED[1]

# --- Setup Calibration & Tracker ---

calibration_filepath = './calibration/calibration_minicam.npz'
camera_matrix, dist_coeffs = load_calibration_data(calibration_filepath)

# Initialize the ArUco tracker from new_aruco_lib.
aruco_tracker = ArucoSingleTracker(
    id_to_find=3,            # marker ID to detect
    marker_size=0.254,       # marker size in meters
    camera_matrix=camera_matrix,
    camera_distortion=dist_coeffs,
    show_video=True          # internal window (if desired)
)

# Set a manual UAV yaw (in radians) for testing conversion.
manual_yaw = math.radians(30)  # 30° yaw; modify as needed

# --- Start the Detection Thread (with show_video disabled) ---
# (GUI calls will be performed in the main thread.)
det_thread = threading.Thread(
    target=aruco_tracker.track,
    kwargs={'loop': True, 'verbose': True, 'show_video': False},
    daemon=True
)
det_thread.start()

print("Tracker running. Press 'q' in the video window to quit.")

# --- Main Loop: Poll Detection Data & Compute Offsets ---
while True:
    with aruco_tracker._lock:
        found = aruco_tracker.latest.get('found', False)
        R_cam = aruco_tracker.latest.get('R_cam', np.eye(3))
        # t_world computed from pose estimation (e.g., -R_cam.T @ tvec)
        t_world = np.array(aruco_tracker.latest.get('t_world', (0.0, 0.0, 0.0)))
        # tvec is the original marker translation vector in camera frame.
        tvec = np.array(aruco_tracker.latest.get('tvec', (0.0, 0.0, 0.0)))
        frame = aruco_tracker.latest.get('frame', None)

    if frame is not None:
        cv2.imshow("Frame", frame)

    if found:
        # Add noise to yaw: simulate flight noise with 1° standard deviation.
        noise = np.random.normal(0, math.radians(1))
        noisy_yaw = manual_yaw + noise
        
        # New method: use marker_to_ned() with noisy yaw.
        new_north, new_east = marker_to_ned(noisy_yaw, R_cam, t_world)
        # Old method: use tvec via camera_to_uav -> uav_to_ne (with fixed yaw).
        x_cam, y_cam, _ = tvec
        x_uav, y_uav = camera_to_uav(x_cam, y_cam)
        old_north, old_east = uav_to_ne(x_uav, y_uav, manual_yaw)
        print(f"New offsets [noisy yaw]: North = {-new_north:.2f} m, East = {-new_east:.2f} m | "
              f"Old offsets: North = {old_north:.2f} m, East = {old_east:.2f} m")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.05)

# --- Cleanup ---
aruco_tracker.stop()
det_thread.join()
cv2.destroyAllWindows()
print("Exiting test.")
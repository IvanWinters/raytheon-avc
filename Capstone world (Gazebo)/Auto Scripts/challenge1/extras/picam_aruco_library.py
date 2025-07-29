import sys
import time
import math
import numpy as np
import cv2
import cv2.aruco as aruco
from os import path
from picamera2 import Picamera2


def load_calibration_data(filepath):
    calibration_data = np.load(filepath)
    camera_matrix = calibration_data['camera_matrix']
    dist_coeffs = calibration_data['dist_coeffs']
    return camera_matrix, dist_coeffs


class ArucoSingleTracker:
    def __init__(self,
                 id_to_find,
                 marker_size,
                 camera_matrix,
                 camera_distortion,
                 picamera2_resolution=(640, 480),
                 show_video=False):
        self.id_to_find = id_to_find
        self.marker_size = marker_size
        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion
        self._show_video = show_video
        self._kill = False

        # Rotation flip for Euler extraction
        self._R_flip = np.diag([1.0, -1.0, -1.0], dtype=np.float32)

        # ArUco detector
        self._aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_100)
        self._parameters = aruco.DetectorParameters()

        # Picamera2 setup
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "BGR888", "size": picamera2_resolution}
        )
        self.picam2.configure(config)
        self.picam2.start()

        # Prepare video window
        self.font = cv2.FONT_HERSHEY_PLAIN
        self._t_read = time.time()
        self._t_detect = self._t_read
        self.fps_read = 0.0
        self.fps_detect = 0.0

    def _rotationMatrixToEulerAngles(self, R):
        Rt = R.T
        if np.linalg.norm(np.eye(3) - Rt @ R) > 1e-6:
            raise ValueError("Invalid rotation matrix")
        sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def _update_fps_read(self):
        t = time.time()
        self.fps_read = 1.0 / (t - self._t_read)
        self._t_read = t

    def _update_fps_detect(self):
        t = time.time()
        self.fps_detect = 1.0 / (t - self._t_detect)
        self._t_detect = t

    def stop(self):
        self._kill = True

    def track(self, loop=True, verbose=False):
        marker_found = False
        x = y = z = 0

        def process_frame(frame):
            nonlocal marker_found, x, y, z
            self._update_fps_read()
            corners, ids, _ = aruco.detectMarkers(frame, dictionary=self._aruco_dict, parameters=self._parameters)
            if ids is not None and self.id_to_find in ids.flatten():
                marker_found = True
                self._update_fps_detect()
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self._camera_matrix, self._camera_distortion)
                rvec, tvec = rvecs[0, 0], tvecs[0, 0]
                x, y, z = tvec
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                pos_cam = -R_tc @ np.matrix(tvec).T
                if verbose:
                    print(f"Marker X={tvec[0]:.1f} Y={tvec[1]:.1f} Z={tvec[2]:.1f} fps={self.fps_detect:.0f}")
                if self._show_video:
                    aruco.drawDetectedMarkers(frame, corners)
                    cv2.putText(frame, f"Marker pos x={x:4.0f} y={y:4.0f} z={z:4.0f}", (10, 80), self.font, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Camera pos x={pos_cam[0,0]:4.0f} y={pos_cam[1,0]:4.0f} z={pos_cam[2,0]:4.0f}", (10, 110), self.font, 1, (0, 255, 0), 2)
            return frame

        while not self._kill:
            frame = self.picam2.capture_array()
            frame = process_frame(frame)
            if self._show_video:
                cv2.imshow('frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if not loop:
                break

        if self._show_video:
            cv2.destroyAllWindows()
        return (marker_found, x, y, z)


if __name__ == "__main__":
    id_to_find = 1
    marker_size = 0.254  # meters
    calib_filepath = 'calibration_logi.npz'
    camera_matrix, dist_coeffs = load_calibration_data(calib_filepath)

    tracker = ArucoSingleTracker(
        id_to_find=id_to_find,
        marker_size=marker_size,
        camera_matrix=camera_matrix,
        camera_distortion=dist_coeffs,
        show_video=True
    )
    tracker.track(verbose=True)

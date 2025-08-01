"""
Here are the defined reference frames:

TAG:
                A y
                |
                |
                |tag center
                O---------> x

CAMERA:
                X--------> x
                | frame center
                |
                |
                V y

F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis

The attitude of a generic frame 2 respect to a frame 1 can be obtained by calculating euler(R_21.T)

We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
from os import path, sys


def load_calibration_data(filepath):
    calibration_data = np.load(filepath)
    camera_matrix = calibration_data['camera_matrix']
    dist_coeffs = calibration_data['dist_coeffs']
    return camera_matrix, dist_coeffs

class ArucoSingleTracker():
    def __init__(self,
                 id_to_find,
                 marker_size,
                 camera_matrix,
                 camera_distortion,
                #  camera_size=[3840, 2160],
                 camera_size=[1920, 1080],
                 show_video=False):
        
        self.id_to_find         = id_to_find
        self.marker_size        = marker_size
        self._show_video        = show_video
        
        self._camera_matrix     = camera_matrix
        self._camera_distortion = camera_distortion
        
        self.is_detected        = False
        self._kill              = False
        
        #--- 180 deg rotation matrix around the x axis
        self._R_flip = np.zeros((3,3), dtype=np.float32)
        self._R_flip[0,0] = 1.0
        self._R_flip[1,1] = -1.0
        self._R_flip[2,2] = -1.0

        #--- Define the aruco dictionary
        self._aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_100)
        self._parameters  = aruco.DetectorParameters()

        #--- Capture the videocamera (this may also be a video or a picture)
        self._cap = cv2.VideoCapture(0)
        #-- Set the camera size as the one it was calibrated with
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

        #-- Font for the text in the image
        self.font = cv2.FONT_HERSHEY_PLAIN

        self._t_read    = time.time()
        self._t_detect  = self._t_read
        self.fps_read   = 0.0
        self.fps_detect = 0.0    

    def _rotationMatrixToEulerAngles(self, R):
        # Calculates rotation matrix to euler angles.
        # The result is the same as MATLAB except the order
        # of the euler angles ( x and z are swapped ).
    
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6        
        assert isRotationMatrix(R)

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
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

    def track(self, loop=True, verbose=False, show_video=None):
        
        self._kill = False
        if show_video is None:
            show_video = self._show_video
        
        marker_found = False
        x = y = z = 0
        
        while not self._kill:
            
            #-- Read the camera frame
            ret, frame = self._cap.read()
            self._update_fps_read()
            
            #-- Convert to gray scale
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(
                frame,
                dictionary=self._aruco_dict, 
                parameters=self._parameters,
                
            )
                            
            if ids is not None and self.id_to_find in ids[0]:
                marker_found = True
                self._update_fps_detect()
                #-- ret = [rvec, tvec, ?]
                #-- rvec: attitude of the marker w.r.t. camera frame
                #-- tvec: position of the marker in camera frame
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
                
                x, y, z = tvec[0], tvec[1], tvec[2]

                #-- Draw the detected marker and its axis
                aruco.drawDetectedMarkers(frame, corners)
                #aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, 10)

                #-- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T

                #-- Get the marker's attitude (in Euler angles) with the proper flip
                roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip * R_tc)

                #-- Get the camera's position with respect to the marker
                pos_camera = -R_tc * np.matrix(tvec).T
                
                if verbose:
                    print("Marker X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f" %
                          (tvec[0], tvec[1], tvec[2], self.fps_detect))

                if show_video:
                    #-- Display marker position on the frame
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
                    cv2.putText(frame, str_position, (0, 100), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)        
                    
                    #-- Display marker attitude on the frame
                    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                        math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker)
                    )
                    cv2.putText(frame, str_attitude, (0, 150), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    #-- Display camera position on the frame
                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                        pos_camera[0], pos_camera[1], pos_camera[2]
                    )
                    cv2.putText(frame, str_position, (0, 200), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    #-- Get and display the camera's attitude using the correct function call
                    roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip * R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                        math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera)
                    )
                    cv2.putText(frame, str_attitude, (0, 250), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                if verbose:
                    print("Nothing detected - fps = %.0f" % self.fps_read)
            
            if show_video:
                #--- Display the frame
                cv2.imshow('frame', frame)
                #--- Use 'q' to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self._cap.release()
                    cv2.destroyAllWindows()
                    break
            
            if not loop:
                return (marker_found, x, y, z)
            

if __name__ == "__main__":

    #--- Define Tag
    id_to_find  = 1
    marker_size = 0.254  # [m]

    #--- Load camera calibration data from the NPZ file
    # calibration_filepath = 'calibration_elp.npz'
    calibration_filepath = 'calibration_logi.npz'
    camera_matrix, dist_coeffs = load_calibration_data(calibration_filepath)
    
    #--- Initialize the tracker with the loaded calibration data
    aruco_tracker = ArucoSingleTracker(
        id_to_find=id_to_find, 
        marker_size=marker_size, 
        show_video=True, 
        camera_matrix=camera_matrix, 
        camera_distortion=dist_coeffs
    )
    
    aruco_tracker.track(verbose=True)

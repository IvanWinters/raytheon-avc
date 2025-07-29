import cv2
from cv2 import aruco
import numpy as np
import os


CHARUCO_BOARD_ROWS = 5
CHARUCO_BOARD_COLS = 9
SQUARE_SIZE = 0.04
MARKER_SIZE = 0.025  

#4 now use arUco original
aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

charuco_board = cv2.aruco.CharucoBoard(
    (CHARUCO_BOARD_COLS, CHARUCO_BOARD_ROWS),
    SQUARE_SIZE,
    MARKER_SIZE,
    aruco_dict
)
def create_board():
    board_size = (1000, 1500)  # Output image size in pixels (width, height)
    charuco_board_image = charuco_board.generateImage(board_size)

    # Save the board image to a file
    cv2.imwrite("charuco_board.png", charuco_board_image)

    # Display the board image
    cv2.imshow("Charuco Board", charuco_board_image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

CharucoDetector = cv2.aruco.CharucoDetector(charuco_board)

charuco_corners = None
charuco_ids = None
marker_corners = None
marker_ids = None
all_charuco_corners = []
all_charuco_ids = []
image_size = (1920,1080)



def charuco_read(images):
   
    for image in images:
        print("Processing {0}".format(image))
        image_data = cv2.imread(image)
        if image_data is None:
            exit("no image")
        charuco_corners, charuco_ids, marker_corners, marker_ids = CharucoDetector.detectBoard(image_data)
       #     interpolated = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image_data, charuco_board)
        if charuco_corners is not None and charuco_ids is not None:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)

        print(charuco_corners)
        print(charuco_ids)
        print(marker_corners)
        print(marker_ids)
        print()

def charuco_calibrate():
        object_points = []  #3D points in Charuco board coordinate space
        image_points = []   #Corresponding 2D points in image coordinate space

        
        for charuco_corners, charuco_ids in zip(all_charuco_corners, all_charuco_ids):
            if charuco_corners is not None and charuco_ids is not None:
                obj_pts, img_pts = charuco_board.matchImagePoints(charuco_corners, charuco_ids)
                object_points.append(obj_pts)
                image_points.append(img_pts)

          # Perform camera calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,  # 3D points
        image_points,   # 2D points
        image_size,     # (width, height)
        None,           # Initial camera matrix
        None            # Initial distortion coefficients
        )

        # Output results
        print("Calibration Successful:", ret)
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)
        np.savez('calibration_data_live.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)


datadir = "./output/"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png") ])
order = np.argsort([int(p.split(".")[-2].split("_")[-1]) for p in images])
images = images[order]
#print(images)
charuco_read(images)
charuco_calibrate()

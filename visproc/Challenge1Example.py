"""
Lissajous search mission. The drone's starting point is random. The only information we need is the center of the field. 

Lissajous curves are defined by two parametric equations:
[x,y] = < Ax*sin(wx * t + phix), Ay*sin(wy * t + phiy) >
    Ax = Amplitude in X-direction
    Ay = Amplitude in Y-direction

    wx = Angular frequency in X-dir
    wy = Angular frequency in Y-dir

    phix = Phase shift in X-dir
    phiy = Phase shift in Y-dir


Lissajous automatically bounds the field dimensions with Ax and Ay. A geofence will still be created 
for extra caution and fail-safe behavior. Adjusting the six parameters leads to infinitely many patterns.
If ratio rw = wx/wy is rational, the curve is cyclical (repeating). Irrational -> the pattern doesn't ever 
repeat.

Latitude, longitude, altitude, heading
"""

#############DEPENDENCIES#############
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import numpy as np
import math
import argparse

import cv2
import numpy as np
import time
import threading
import queue

#Used to track execution time throughout programa for logging, do:       time.time() - start_time
start_time = time.time()
print("Program start")


"""

Lissajous search mission. The drone's starting point is random. The only information we need is the center of the field.

Lissajous curves are defined by two parametric equations:
[x,y] = < Ax*sin(wx * t + phix), Ay*sin(wy * t + phiy) >
    Ax = Amplitude in X-direction
    Ay = Amplitude in Y-direction

    wx = Angular frequency in X-dir
    wy = Angular frequency in Y-dir

    phix = Phase shift in X-dir
    phiy = Phase shift in Y-dir


Lissajous automatically bounds the field dimensions with Ax and Ay. A geofence will still be created
for extra caution and fail-safe behavior. Adjusting the six parameters leads to infinitely many patterns.
If ratio rw = wx/wy is rational, the curve is cyclical (repeating). Irrational -> the pattern doesn't ever
repeat.

latitude, longitude, altitude, heading

Navigation has 2-3 different phases:
1. Scout search pattern
2. Breaking out of pattern to maneuver to marker
3. And if Package Delivery involved, communicate waypoint to then maneuver to marker (one extra step)
"""

########CONSTANTS#############
pi = math.pi
T_STEP = pi/8 # defines granularity of curve with more (x,y) coordinates
T_MAX = 64 # how many time points we want from 0 - T_MAX (though it doesn't have to be 0) & should be enough for the challenge duration
AMP_X = 10 # meters
AMP_Y = 10 # field dimensions are 90ft x 90ft ~ approx. 27.4 m. The curve will be slightly less than boundary to not trigger geofence
W_X = 1
W_Y = 2
PHI_X = pi/2
PHI_Y = 0
SCOUT_ALT = 10 # meters ~ approx. 33 ft
CENTER_LAT = -35.363262
CENTER_LONG = 149.165237 # Default Gazebo/SITL params
EARTH_RADIUS = 6378.137 # km

#############CONNECTION#############
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

if connection_string is not None:
    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    scout = connect(connection_string, wait_ready=True)


#####SETTING UP THE LISSAJOUS CURVE#######
def lissajous_search():
    print("Lissajous curve search pattern") #let's make a repeating figure 8 for starters...

    """

    The array represents the Cartesian plane with the origin being in the middle of the array.

    +-------^-------+
    |       |       |
    |       |       |
    <-------+------->
    |       |       |
    |       |       |
    +---------------+

    The array will need to be twice the amplitudes in both directions + 1 to include the four quadrants.
    The amplitudes will only count half, and are therefore known as half-amplitudes.

    y-coordinates corrspond to rows. x-coordinates correspond to the columns.

    To access the cell that corresponds to the appropriate (x,y) points, follow this mapping:

    cell array[row] = -Y_point + AMP_Y
    cell array[column] = X_point + AMP_X

    So, an (x,y) pair maps to array[-Y_point + AMP_Y][X_point + AMP_X].
    and the origin (0,0), center of field, is array[AMP_Y][AMP_X]
    """

   # cmds = scout.commands
  #  cmds.download() # Download current list of commands FROM drone
  #  cmds.wait_ready() # wait until download is complete

   # print(" Clear any existing commands")
 #   cmds.clear() # Clear list before adding new ones

    rows = 2*AMP_Y + 1
    columns = 2*AMP_X + 1 # store longitude and latitude coordinates every 1 meter apart

    met_to_deg = (1 / ((pi/180) * EARTH_RADIUS)) / 1000 # converting between degrees to meters, constant

    # latitude is approx. constant at all points of the Earth
    # new latitude = original latitude + translation_meters * meters_to_degrees
    # positive translation -> move up
    # negative translation -> move down
    np.set_printoptions(precision=10)
    latitude_arr = np.arange(rows*columns, dtype=np.float64).reshape(rows, columns)
    for row in range(0, rows):
        for col in range(0, columns):
            latitude_arr[row][col] = CENTER_LAT +  (AMP_Y - row) * met_to_deg
            #print(latitude_arr[row][col])

    # longitude varies with latitude degrees
    # new longitude = original longitude + (translation_mters * meters_to_degrees / cos(original long. * pi/180))
    # positive translation -> move left
    # negative translation -> move down
    longitude_arr = np.arange(rows*columns, dtype=np.float64).reshape(rows, columns)
    for row in range(0, rows):
        for col in range(0, columns):
            longitude_arr[row][col] = CENTER_LONG + ((AMP_X - col) * met_to_deg)/(math.cos(CENTER_LAT * (pi/180)))
            #print(longitude_arr[row][col])

    print("Add lissajous waypoints.")
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    #cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, SCOUT_ALT))
    waypoints = []
    for T_RANGE in range(T_MAX):
        X_point = math.ceil(AMP_X * math.sin(W_X * T_STEP * T_RANGE + PHI_X))
        Y_point = math.ceil(AMP_Y * math.sin(W_Y * T_STEP * T_RANGE + PHI_Y))

        # Ensure indices are within bounds
        y_index = max(0, min(rows - 1, -Y_point + AMP_Y))
        x_index = max(0, min(columns - 1, X_point + AMP_X))

        lat = float(latitude_arr[y_index][x_index])
        lon = float(longitude_arr[y_index][x_index])

        # Append waypoint as a tuple (latitude, longitude)
        waypoints.append((lat, lon))
    print("YAY Lissajous curve made ^_^")

    print(" Upload search pattern to vehicle")
    #cmds.upload()
    return waypoints

"""
Boundary points that define field at Xelevate location (for geofence and breaches)

A.------- B.
|         |
|         |
|         |
C.------- D.

boundary_locations = {
    "point_A": LocationGlobalRelative(39.234743, -77.546500, 10),
    "point_B": LocationGlobalRelative(39.234718, -77.546199, 10),
    "point_C": LocationGlobalRelative(39.234519, -77.54653, 10),
    "point_D": LocationGlobalRelative(39.234501, -77.546227, 10),
}
"""

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat)+(dlong*dlong))*1.113195e5 # 1.113195e5 is the number of metres per degree of lat/long
    
def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = scout.commands.next
    if nextwaypoint==0:
        return None
    missionitem=scout.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(scout.location.global_frame, targetWaypointLocation)
    return distancetopoint

def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not scout.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    scout.mode = VehicleMode("GUIDED")
    scout.armed = True

    while not scout.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    scout.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", scout.location.global_relative_frame.alt)      
        if scout.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def main():

    #Multithread the aruco detection here using tracker class
    tracker = aruco_tracker()
    detection_thread = threading.Thread(target=tracker.search)
    detection_thread.start()

    idx = 0
    waypoints = lissajous_search()
    curr_time = time.time() 
    while True:
        if tracker.frame_queue:
            frame = tracker.frame_queue.get()            
            cv2.imshow('frame', frame)
            cv2.waitKey(1) 

        #Checks if a previously valued variable of None has changed values. In other words, it has found a correct marker
        if tracker.found_marker is not None:
            detection_thread.join() #End thread
            print("Marker found breaking loop")
            break
            goto_marker() #We have found the marker so start moving towards it todo

        if cv2.waitKey(1) == ord('q'):
            print("SHUTDOWN DETECTED")
            tracker.shutdown = True
            detection_thread.join() #End thread
            break

        if idx+1 >= len(waypoints):
            print("end of waypoints reached")
            break
        
        if time.time()-curr_time >= 1:   
            curr_time = time.time()
            print("Going to waypoint", waypoints[idx])
            idx += 1


    #Close vehicle object before exiting script
    print("End Mission")

    tracker.cap.release()
    cv2.destroyAllWindows()
    



class aruco_tracker():
    #Aruco Detector class:
    #Competition day ID is unknown until competition day, assuming '2' for testing
    #We are using aruco 6x6 per competition spec
    #found marker will be checked by main script in main() for any value that isnt None
    def __init__(self):
        self.TARGET_ID = 2
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)                 #Opencv detection parameters for detecting aruco markers
        self.parameters = cv2.aruco.DetectorParameters()                                            #
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)                   #
        self.found_marker = None                                                                    #The found aruco marker of self.TARGET_ID
        self.frame_queue = queue.Queue(maxsize=1)                                                   #A queue of frames to show, max size one because you dont want a line of frames to pile up
        self.cap = cv2.VideoCapture(0)                                                              #The video capture for the tracker should not chnage
        self.shutdown = False                                                                       #For manual shut down of tracker class, set to True

        
    #Simple search, only looks for the marker and does not do any POSE estimation or ML estimation    
    def search(self):


        while self.found_marker is  None and self.shutdown is False:
            ret, frame = self.cap.read()

            if not ret:
                print("Failed to grab frame")
                continue

            #detect markers in the frame
            corners, ids, rejected = self.detector.detectMarkers(frame)
                
            #Currently detected markers including non-target ones for logging purposes
            detected_markers = []

            #if any markers are detected...
            if ids is not None:
                for i in range(len(ids)):
                    #draw the id associated with the detected marker
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[ids[i][0]]], dtype=np.int32))

                    #Get the center of the markere based on the average coordinates of the corner
                    #This is the center in pixels and not any physical dimension
                    x = (corners[i-1][0][0][0] + corners[i-1][0][1][0] + corners[i-1][0][2][0] + corners[i-1][0][3][0]) / 4
                    y = (corners[i-1][0][0][1] + corners[i-1][0][1][1] + corners[i-1][0][2][1] + corners[i-1][0][3][1]) / 4
                    center = (float(x),float(y))

                    if ids[i][0] == self.TARGET_ID:
                        #convert marker corners to integer coordinates
                        int_corners = np.int32(corners[i])
                        #use integer coordinates to draw a red box around marker
                        cv2.polylines(frame, [int_corners], isClosed=True, color=(0, 0, 255), thickness=5)
                                        
                        #print(f"Distance to marker {TARGET_ID}: {distance:.2f} meters")
                        found_target = {
                            "real_time" : time.time(),
                            "program_time" : time.time() - start_time,                    
                            "target_id" : float(ids[i][0]),
                            "center" : center,
                            "is_target" : True
                        } 

                        #Print to STDOUT
                        detected_markers.append(found_target)
                        
                        #Set found marker to target which will be send to main script to block
                        self.found_marker = found_target

                        #draw the coordinate frame axes and print distance onto the frame
                        cv2.putText(frame, f"DropZone Found {self.TARGET_ID}", (10, 70), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        break
                    else: 
                        found_target = {
                            "real_time" : time.time(),
                            "program_time" : time.time() - start_time,                    
                            "target_id" : ids[i][0],
                            "center" : center,
                            "is_target" : False
                        } 
                        cv2.putText(frame, "Non-DropZone(s)", (10, 110), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)  
                        
                        detected_markers.append(found_target)


                #print(detected_markers)
                                
            self.frame_queue.put(frame)
            #Dont use due to thread safety
            #cv2.imshow('frame', frame)

            

    
if __name__ == "__main__":

    main()

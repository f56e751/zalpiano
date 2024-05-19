import cv2
import cv2.aruco as aruco
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class Aruco():
    def __init__(self, aruco_dict_type=cv2.aruco.DICT_4X4_250):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = aruco.DetectorParameters_create()
        self.marker_heading = 0
        self.distance = 200
        self.centers = {}
        self.initialize_distance = True

        self.vector0to2 = [0,0]
        self.center = [0,0]
        self.last_center = [0,0]

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        self.processCorner(corners, ids, frame)

    def returnCenter(self):
        return self.center
    
    def returnMarkerHeading(self):
        return self.marker_heading

    def initializeDistance(self, corners, ids):
        visible_markers = ids.flatten()
        centerDict = {}
        for i, marker_id in enumerate(visible_markers):
            center = corners[i][0].mean(axis=0)
            centerDict[marker_id] = center

        # Calculate the direct distances between visible adjacent markers
        for i in range(len(visible_markers)):
            if i < len(visible_markers) - 1:  # Ensure not out of range
                marker1 = visible_markers[i]
                marker2 = visible_markers[(i + 1) % len(visible_markers)]
                if {marker1, marker2} in [{0, 1}, {1, 2}, {2, 3}, {3, 0}]:
                    vector = centerDict[marker2] - centerDict[marker1]
                    self.distance = np.linalg.norm(vector) # * 2 ** (1 / 2)
                    print(f"Distance between marker {marker1} and {marker2}: {self.distance}")

        # If both diagonal markers are visible, calculate their distances
        if 0 in centerDict and 2 in centerDict:
            self.vector0to2 = centerDict[2] - centerDict[0]
            self.distance = np.linalg.norm(self.vector0to2)
            print("Distance between markers 0 and 2 initialized.")

        if 1 in centerDict and 3 in centerDict:
            self.vector1to3 = centerDict[3] - centerDict[1]
            self.distance = np.linalg.norm(self.vector1to3)
            print("Distance between markers 1 and 3 initialized.")


    def getCenter(self, visible_markers, corners):
        visible_markers_num = len(visible_markers)
        centerArr = []
        centerDict = {}
        center = np.array([0.0, 0.0])
        # Processing each visible marker
        for i in range(len(visible_markers)):
            # Calculate the mean of corners to find the center of each marker
            # Assuming 'corners' is indexed by the same order as 'visible_markers'
            singlecenter = corners[i][0].mean(axis=0)
            centerArr.append(singlecenter)
            centerDict[visible_markers[i]] = singlecenter

        # Handling different numbers of visible markers
        if visible_markers_num == 4:
            center = np.mean(centerArr, axis=0)
        elif visible_markers_num == 3:
            # Handle cases with three markers where two specific markers need to be averaged
            pair_keys = [(0, 2), (1, 3)]
            for key_pair in pair_keys:
                if all(k in visible_markers for k in key_pair):
                    center = np.mean([centerDict[key_pair[0]], centerDict[key_pair[1]]], axis=0)
                    break
        elif visible_markers_num == 2:
            # Averaging centers for pairs or using predefined vectors for calculation
            for key_pair in [(0, 2), (1, 3)]:
                if all(k in visible_markers for k in key_pair):
                    center = np.mean([centerDict[key_pair[0]], centerDict[key_pair[1]]], axis=0)
                    break
            else:
                # If not a standard pair, apply vector calculations
                centers = [self.getSingleArucoCenter(marker, centerDict[marker]) for marker in visible_markers]
                center = np.mean(centers, axis=0)
        elif visible_markers_num == 1:
            # Directly use the single marker's calculated center
            marker = visible_markers[0]
            center = self.getSingleArucoCenter(marker, centerDict[marker])
        else:
            center = self.preCenter
            print("No valid markers detected to calculate center")

        self.updateCenterVector(centerDict)
        self.preCenter = center
        return center


    def updateCenterVector(self,centerDict):
        if 0 in centerDict and 2 in centerDict:
            self.vector0to2 = np.subtract(centerDict[2] , centerDict[0])
            self.distance = np.linalg.norm(self.vector0to2) 
        if 1 in centerDict and 3 in centerDict:
            self.vector1to3 = np.subtract(centerDict[3] , centerDict[1])
            self.distance = np.linalg.norm(self.vector1to3) 

    def processCorner(self, corners, ids, frame):
        if self.initialize_distance:
            if ids is not None:
                self.initializeDistance(corners, ids)
                self.initialize_distance = False
        
        if ids is not None:
            visible_markers = ids.flatten()  # Flatten the array of IDs
            # print(f"Visible markers: {visible_markers}")  # Print visible marker IDs
            self.center = self.getCenter(visible_markers, corners)
            self.last_center = self.center

            for i, marker_id in enumerate(visible_markers):
                if 0 <= marker_id <= 3:
                    self.getMarkerHeading(marker_id, corners[i], frame)

        else:
            print("No markers detected.")
            if self.last_center is not None:
                # cv2.circle(frame, (int(self.last_center[0]), int(self.last_center[1])), 10, (0, 0, 255), -1)
                pass

    
    def getSingleArucoCenter(self, marker, center):
        # Convert heading to radians for calculation
        # marker_heading = self.markerHeading
        heading_radians = np.deg2rad(self.marker_heading)

        # Assume we have a default distance from the marker to the center we want to calculate
        distance = self.distance  # Adjust distance as required

        heading_radians = np.deg2rad(self.marker_heading)

        # Calculate the new center coordinates based on the marker heading
        # print(f"self.marker_heading is {self.marker_heading}")
        # print(f"heading_radians is : {heading_radians}")
        dx = distance * np.cos(heading_radians) / 2
        dy = distance * np.sin(heading_radians) / 2

        if marker == 0:
            adjusted_center = center + np.array([-dy, +dx])
        elif marker == 1:
            adjusted_center = center + np.array([+dx, +dy])
        elif marker == 2:
            adjusted_center = center + np.array([dy, -dx])
        elif marker == 3:
            adjusted_center = center + np.array([-dx, -dy])
        else:
            # Default case if marker ID is unexpected
            adjusted_center = center
            print(f"Warning: Unexpected marker ID {marker}, using center as adjusted center.")



        # print(f"{marker} marker center is {adjusted_center}")

        return adjusted_center


        
    def getMarkerHeading(self, marker_id, corner, frame):
        aruco.drawDetectedMarkers(frame, [corner])

        c = corner[0]
        center = c.mean(axis=0)
        # print(f"marker id: {marker_id}, center: {center}")
        X, Y, Z = self.image_point_to_world(center[0], center[1])

        vector = c[1] - c[0]
        angle = np.arctan2(vector[1], vector[0])
        self.marker_heading = np.degrees(angle)

    def image_point_to_world(self, x, y):
        # Placeholder values for the intrinsic camera matrix and distortion coefficients
        # You should replace these with actual calibration data
        fx = 600  # Focal length in x
        fy = 600  # Focal length in y
        cx = 640  # Camera principal point x-coordinate
        cy = 400  # Camera principal point y-coordinate

        # Assuming a simple pinhole camera model without distortion
        # and a fixed Z distance (depth) from the camera for simplicity
        Z = 3.0  # Depth from the camera to the marker plane in your unit of choice

        # Convert from image coordinates to normalized device coordinates
        X = (x - cx) * Z / fx
        Y = (y - cy) * Z / fy

        return X, Y, Z
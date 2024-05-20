# from .camera import Camera
# from .aruco import Aruco
# from .human import HumanTracker
import sys
import os

# boxing_2_0_0 디렉토리의 부모 디렉토리를 sys.path에 추가
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from boxing_2_0_0 import Camera, Aruco, HumanTracker, Tilt, PunchData, Punch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float64

import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs

import subprocess
import re

import collections
import time
import math


##### at initail frame, solve the problem when only 1 or 2 aruco is visible (no error)

class IntegratedSystem(Node):
    def __init__(self):



        

        ################ pre calibrate color ##############
        self.green = np.array([ 78, 255, 139])   # 
        self.blue = np.array([107, 223, 153])
        self.yellow = np.array([ 29, 244, 254])
        
        self.green = np.array([ 79, 244, 174]) # A 105
        self.blue = np.array([105, 227, 180])
        self.yellow = np.array([ 31, 233, 254])
        self.ispreCalibrete = True
        #####################################################


        super().__init__('integrated_system')
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.action_publisher = self.create_publisher(Float64, 'optimal_action', 10)
        self.relative_heading_publisher = self.create_publisher(Float64, 'relative_heading', 10)

        ###### camera setting #############
        camera = Camera()
        fps = 120
        width, height = 1280, 800
        camera.setFps(fps)
        camera.setFrame(width, height)
        self.camera = camera.getCamera()
        self.frame_count = 0
        self.last_time = 0

        self.frame = None
        ####################################

        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters_create()

        self.Aruco = Aruco()
        self.HumanTracker = HumanTracker()
        self.Tilt = Tilt()
        self.PunchData = PunchData()

        # self.pipeLength = 100
        # self.Tilt.initializePipeLength(self.pipeLength)

        self.optimal_action = None
        self.preOptimal_action = None

        self.humanPosition = (0,0)
        self.humanVelocity = (0,0)
        self.humanSpeed = 0
        self.humanDirection = 0
        

        self.rightPosition = (0,0)
        self.rightVelocity = (0,0)
        self.rightSpeed = 0
        self.rightDirection = 0

        self.leftPosition = (0,0)
        self.leftVelocity = (0,0)
        self.leftSpeed = 0
        self.leftDirection = 0

        self.relative_heading = 0
        self.person_heading = 0
        self.left_heading = 0
        self.right_heading = 0
        self.leftHeadingVector = [0,0]
        self.rightHeadingVector = [0,0]


        ######## 이 값을 이용해서 로직 추가하기!!! 뺄떄 vel danger 인식 X ######  -> 안해도 될듯
        self.isLeftComingClose = False
        self.isRightComingCLose = False
        #####################################################################
    
        self.preRightPosition = [0,0]
        self.preLeftPosition = [0,0]
        

        self.isPunch = False
        self.preOptimalPosition = [0,0]



        # Initialization for both hands
        self.record_left_hand = False
        self.left_hand_positions = []
        self.record_right_hand = False
        self.right_hand_positions = []


    def timer_callback(self):
        current_time = time.time()
        self.frame_count += 1
        elapsed_time = current_time - self.last_time

        if elapsed_time > 1.0:  # Update FPS every second
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.last_time = current_time
            # print(f"FPS: {self.fps:.2f}")

        ret, self.frame = self.camera.read()
        if ret:
            if not self.HumanTracker.isCalibrationDone():
                if self.ispreCalibrete:
                    self.preCalibrete()
                self.getHumanPosition()
                self.show_frame()
                return
            
            self.process_aruco()

            

            self.getHumanPosition()
            self.getPersonHeading()
            self.judgeTilt()


            self.punblish()



            ########### draw on frame ################
            
            self.drawOnFrame()
            self.getHitPointUnitTest()
            self.show_frame()


            ###  이제 heading에 따라 피하는 로직 추가

        else:
            self.get_logger().error('Failed to capture frame.')


    def drawOnFrame(self):
        self.showOptimalAction()
        self.showPossibleAction()

        self.showIsPunch()
        # self.showDangerReason() # TODO tilt_2_0 update this method
        self.showPunchVel()

        self.putTextOnFrame(f"isLeftComingCLose: {self.isLeftComingClose}", (50,300),1, (255,0,0))
        self.putTextOnFrame(f"isRightComingCLose: {self.isRightComingClose}", (50,330),1, (255,0,0))

        self.putTextOnFrame(f"right direction: {self.rightDirection}", (50,400),1, (100,100,200))
        self.putTextOnFrame(f"left direction: {self.leftDirection}", (50,430),1, (100,100,200))

        self.putTextOnFrame(f"person heading: {self.person_heading: .2f}", (50,600),1, (100,255,0))
        self.putTextOnFrame(f"right heading: {self.right_heading: .2f}", (50,630),1, (100,255,0))
        self.putTextOnFrame(f"left heading: {self.left_heading: .2f}", (50,660),1, (100,255,0))
        self.putTextOnFrame(f"marker heading: {self.markerHeading: .2f}", (50,690),1, (100,255,0))
        
        self.putTextOnFrame(f"fps: {self.fps}", (1000,30),1, (255,0,0))

        self.draw_circle_on_frame(self.humanPosition,10,(0,255,0),2)
        self.draw_circle_on_frame(self.rightPosition,10,(0,255,0),2)
        self.draw_circle_on_frame(self.leftPosition,10,(0,255,0),2)
        self.draw_circle_on_frame(self.center,10,(0,0,255),2)



        self.showHandPositions('right')


        self.showPunchType()
        # print(f"main.drawOnFrame() -> np.around(self.Tilt.getRadius()), is {int(np.around(self.Tilt.getRadius()))}")
        self.draw_circle_on_frame(self.center, int(np.around(self.Tilt.getRadius())), (0,0,255), 2)



    def punblish(self):
        msg = Float64()
        if self.optimal_action != None:
            msg.data = float(self.optimal_action)
            self.action_publisher.publish(msg)
        elif self.optimal_action == None:
            self.optimal_action = float(1000.0)
            msg.data = float(self.optimal_action)
            self.action_publisher.publish(msg)


        heading_msg = Float64()
        heading_msg.data = float(np.deg2rad(self.relative_heading))
        # print(self.relative_heading)
        self.relative_heading_publisher.publish(heading_msg)





    def showHandPositions(self, hand):
        hand_key = 'left' if hand == 'left' else 'right'
        positions_key = f"{hand_key}_hand_positions"
        record_key = f"record_{hand_key}_hand"
        
        # Check if we are recording for the specified hand
        if getattr(self, record_key):
            # Append the current hand position
            current_position = getattr(self, f"{hand_key}Position")
            getattr(self, positions_key).append(current_position.copy())

        # Draw all recorded positions for the specified hand
        if getattr(self, record_key):
            for pos in getattr(self, positions_key):
                self.draw_circle_on_frame(pos, 10, (255, 0, 0), -1)

        # Check for key presses to toggle recording
        key = cv2.waitKey(1) & 0xFF
        if key == ord('g'):
            if getattr(self, record_key):
                # Clear positions when stopping recording
                getattr(self, positions_key).clear()
            setattr(self, record_key, not getattr(self, record_key))
        elif key == 27:  # ESC key to exit
            cv2.destroyAllWindows()
            return


    def showIsPunch(self):
        if self.isPunch:
            self.draw_circle_on_frame([50,50],15,(0,0,255),-1)
            
        else:
            self.draw_circle_on_frame([50,50],15,(0,0,255),5)


    def show_frame(self):
        if self.frame is not None:
            cv2.imshow("Integrated System", self.frame)
            cv2.waitKey(1)
        else:
            print("No frame")


        
        

    def process_aruco(self): 
        self.Aruco.detect_markers(self.frame)
        self.center = self.Aruco.returnCenter()
        self.markerHeading = self.Aruco.returnMarkerHeading()


    def getHumanPosition(self):
        self.frame = self.HumanTracker.process_frame(self.frame)
        humaninfoDict = self.HumanTracker.get_color_info('yellow')
        rightinfoDict = self.HumanTracker.get_color_info('blue')
        leftinfoDict = self.HumanTracker.get_color_info('green')

        self.preRightPosition = self.rightPosition
        self.preLeftPosition = self.leftPosition

        self.humanPosition = humaninfoDict['position']
        self.humanSpeed = humaninfoDict['speed']
        self.humanVel = humaninfoDict['velocity']
        self.humanDirection = humaninfoDict['direction']

        self.rightPosition = rightinfoDict['position']
        self.rightSpeed = rightinfoDict['speed']
        self.rightVelocity = rightinfoDict['velocity']
        self.rightDirection = rightinfoDict['direction']

        self.leftPosition = leftinfoDict['position']
        self.leftSpeed = leftinfoDict['speed']
        self.leftVelocity = leftinfoDict['velocity']
        self.leftDirection = leftinfoDict['direction']

        

    def isPunchComingClose(self):
        leftVector = np.subtract(self.leftPosition, self.preLeftPosition)
        rightVector = np.subtract(self.rightPosition, self.preRightPosition)

        self.isLeftComingClose = True if np.dot(leftVector, self.leftHeadingVector) > 0 else False
        self.isRightComingClose = True if np.dot(rightVector, self.rightHeadingVector) > 0 else False

    def judgeTilt(self):
        self.isPunchComingClose()
        currentTime = time.time()
        self.Tilt.initializeLeft(self.leftPosition, self.leftSpeed, self.leftDirection, self.left_heading, self.leftVelocity, currentTime)
        self.Tilt.initializeRight(self.rightPosition, self.rightSpeed, self.rightDirection, self.right_heading, self.rightVelocity, currentTime)
        self.Tilt.initializePersonHeading(self.person_heading)
        self.Tilt.initializeCenter(self.center)
        self.Tilt.initializePersonPosition(self.humanPosition)
        self.Tilt.setPunchTypeDetector()

        self.optimal_action = self.Tilt.detectPunch()
        if self.optimal_action is None:
            self.isPunch = False
            # self.optimal_action = self.preOptimal_action
        else:
            self.isPunch = True
        
        # self.preOptimal_action = self.optimal_action

    def getHitPointUnitTest(self):
        leftHitPoint, rightHitPoint = self.Tilt.getHitPointUnitTest()
        if leftHitPoint != (None or 1000):
            optimalAbsolute = self.calculateDegree(leftHitPoint)
            sampleDistane = 100
            optimalCoordinate = [self.center[0] - sampleDistane * math.cos(optimalAbsolute), self.center[1] - sampleDistane * math.sin(optimalAbsolute)]
            cv2.circle(self.frame, (int(optimalCoordinate[0]), int(optimalCoordinate[1])), 15, (0, 255, 50), -1)

        if rightHitPoint != (None or 1000):
            optimalAbsolute = self.calculateDegree(rightHitPoint)
            sampleDistane = 100
            optimalCoordinate = [self.center[0] - sampleDistane * math.cos(optimalAbsolute), self.center[1] - sampleDistane * math.sin(optimalAbsolute)]
            cv2.circle(self.frame, (int(optimalCoordinate[0]), int(optimalCoordinate[1])), 15, (255, 0, 0), -1)


    def showPunchVel(self):
        self.putTextOnFrame(f"green speed: {self.leftSpeed: .2f}",(50,170),1,(0,255,0))
        self.putTextOnFrame(f"blue speed: {self.rightSpeed: .2f}",(50,190),1,(0,255,0))

    def showPunchType(self):
        Left, Right = self.Tilt.getPunch()
        self.putTextOnFrame(f"left Punch Type: {Left.getPunchType()}",(600,170),1.5,(0,255,0))
        self.putTextOnFrame(f"right Punch Type: {Right.getPunchType()}",(600,210),1.5,(0,255,0))

    def showDangerReason(self):
        if self.Tilt.isDistanceDanger():
            self.putTextOnFrame("Distance Danger!!!!!!!",(50,100))
        if self.Tilt.isVelDanger():
            self.putTextOnFrame("Vel Danger!!!!!!!",(50,150))

    def calculateDegree(self, action):
        ## person heading에서 바꿀까????
        return np.deg2rad(self.person_heading) + action + math.pi / 2


    def showOptimalAction(self):
        
        if self.optimal_action != (None or 1000):
            optimalAbsolute = self.calculateDegree(self.optimal_action)
            optimalAbsolute
            sampleDistane = 100
            optimalCoordinate = [self.center[0] - sampleDistane * math.cos(optimalAbsolute), self.center[1] - sampleDistane * math.sin(optimalAbsolute)]
            cv2.circle(self.frame, (int(optimalCoordinate[0]), int(optimalCoordinate[1])), 15, (0, 0, 255), -1)
            # print(f"optimalCoordinate is {optimalCoordinate}")
            self.preOptimalPosition = optimalCoordinate
        else:
            cv2.circle(self.frame, (int(self.preOptimalPosition[0]), int(self.preOptimalPosition[1])), 10, (0, 0, 255), -1)

    def showPossibleAction(self):
        posibleTilt = self.Tilt.posibleTilt
        sampleDistane = 100
        for action in posibleTilt:
            posibleAbsolute = self.calculateDegree(action)
            coordinate =  [self.center[0] - sampleDistane * math.cos(posibleAbsolute), self.center[1] - sampleDistane * math.sin(posibleAbsolute)]
            cv2.circle(self.frame, (int(coordinate[0]), int(coordinate[1])), 7, (0, 255, 0), -1)


    def preCalibrete(self):
        self.HumanTracker.preCalibrateColor(self.green, self.blue, self.yellow)
        
        
        

    def getPersonHeading(self):
        dx, dy = np.subtract(self.humanPosition, self.center)
        # Calculate heading from the positive x-axis to this vector
        self.person_heading = np.degrees(np.arctan2(dy, dx))
        self.relative_heading = self.person_heading - self.markerHeading

        dx, dy = np.subtract(self.center, self.leftPosition)
        self.left_heading = np.degrees(np.arctan2(dy,dx))
        self.leftHeadingVector = np.array([dx, dy])

        dx, dy = np.subtract(self.center, self.rightPosition)
        self.right_heading = np.degrees(np.arctan2(dy,dx))       
        self.rightHeadingVector = np.array([dx, dy])


    def putTextOnFrame(self, text, position, size = 2, color = (0, 0, 255), thickness = 2):
        """Utility to display text on the frame."""
        cv2.putText(self.frame, text, position, 
                    cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)


    def draw_circle_on_frame(self, center, radius=30, color=(0, 255, 0), thickness=2):
        # print(f"center is {center}")
        # cv2.circle(self.frame, center, radius, color, thickness)
        # if center and all(isinstance(x, int) for x in center):

        cv2.circle(self.frame, (int(center[0]),int(center[1])), radius, color, thickness)


def main(args=None):
    rclpy.init(args=args)
    integrated_system = IntegratedSystem()
    rclpy.spin(integrated_system)
    integrated_system.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

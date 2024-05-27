# from .camera import Camera
# from .aruco import Aruco
# from .human import HumanTracker
# from .tilt_2_0 import Tilt
# from .logPunch import PunchData

from camera import Camera
from aruco import Aruco
from human import HumanTracker
from logPunch import PunchData
from punch import Punch
from punchcost import PunchCost
from coordinateTransformer import CoordinateTransformer
from punchcostfunction import SegmentCostFunction, LineCostFunction, SegmentCostFunction_out0, OppositeSideMaxCostFunction
from point import Point
from optimalAction import OptimalAction
from controlRotation import ControlRotation

import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable


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

from std_msgs.msg import Float32, Float32MultiArray

import time
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy



class CircularActionGenerator:
    def __init__(self, radius, num_points, interval):
        self.radius = radius
        self.num_points = num_points
        self.interval = interval
        self.points = self.generate_circle_points(self.radius, self.num_points)
        self.current_index = 0

    def generate_circle_points(self, radius, num_points):
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append((x, y))
        return points

    def circularAction(self):
        if self.current_index >= self.num_points:
            self.current_index = 0
        point = self.points[self.current_index]
        self.current_index += 1
        return point

    def start(self):
        while True:
            point = self.circularAction()
            print(f"Current Point: {point}")
            time.sleep(self.interval)
            return self.circularAction


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
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.action_publisher = self.create_publisher(Float32MultiArray, 'optimal_action', qos_profile)
        # self.relative_heading_publisher = self.create_publisher(Float32, 'relative_heading', qos_profile)
        # self.maxLength_publisher = self.create_publisher(Float32, 'maxLength', qos_profile)

        self.isMaxLengthSend = False
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
        self.PunchData = PunchData()
        self.PunchCost = PunchCost()
        self.isInitializeCostDone = False
        self.CoordinateTransformer = CoordinateTransformer()
        self.CostFunction = None
        self.SandbagPosition = Point()
        self.CircularActionGenerator = CircularActionGenerator(100,120,0.01)

        maximumPhi = np.deg2rad(25)
        maximumLength = 100

        self.ControlRotation = ControlRotation(maximumPhi, maximumLength)

        criticalDistance = 200
        semiCriticalDistance = 250
        self.OptimalAction = OptimalAction(self.PunchCost, criticalDistance, semiCriticalDistance)


        

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


    
        self.preRightPosition = [0,0]
        self.preLeftPosition = [0,0]
        

        self.isPunch = False
        self.preOptimalPosition = [0,0]

        self.criticalDistance = 200
        self.center = (0,0)



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

            self.calculateCost() # TODO ê·¸ë¦¬ëŠ”ê±°ë¥¼ publish ì´í›„ë¡œ ì˜®ê¸°ê¸°
            self.circle()
            self.punblish()
            ########### draw on frame ################
            
            # self.drawOnFrame()
            self.drawCirclePosition()
            # self.drawGrid()
            self.putTextOnFrame(f"fps: {self.fps}", (100,30),1, (255,0,0))
            # self.update_visualization()
            
            self.show_frame()


            ###  ì´ì œ headingì— ë”°ë¼ í”¼í•˜ëŠ” ë¡œì§ ì¶”ê°€

        else:
            self.get_logger().error('Failed to capture frame.')

    def circle(self):
        self.circleX, self.circleY = self.CircularActionGenerator.circularAction()

    def drawCirclePosition(self):
        angle = np.radians(self.person_heading - 90)
        transformed_point = self.CoordinateTransformer.transform(self.center, angle, [self.circleX, self.circleY])
        self.draw_circle_on_frame(transformed_point, 10, (0, 255, 0), -1)  

    def draw_line_with_gradient(self,img, pt1, pt2, max_distance=100):
        # Drawing a basic line
        cv2.line(img, pt1, pt2, (255, 0, 0), 2)

        # Create a blank image with the same dimensions
        mask = np.zeros_like(img)

        # Compute normal vector to the line
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]
        length = np.sqrt(dx**2 + dy**2)
        if length == 0:
            return img  # to avoid division by zero if points coincide

        normal = (-dy / length, dx / length)

        # For each point in the mask, calculate distance from the line
        for y in range(img.shape[0]):
            for x in range(img.shape[1]):
                # Point-line distance
                distance = abs(normal[0] * (x - pt1[0]) + normal[1] * (y - pt1[1]))
                if distance <= max_distance:
                    # Scale color based on distance
                    intensity = 255 - int(255 * (distance / max_distance))
                    cv2.circle(mask, (x, y), 1, (intensity, intensity, 255), -1)

        # Blend the original image and the mask
        alpha = 0.5
        cv2.addWeighted(src1=img, alpha=1-alpha, src2=mask, beta=alpha, gamma=0, dst=img)

        return img
    
    def calculateCost(self):

        if not self.isInitializeCostDone:
            # self.CostFunction = SegmentCostFunction("exponential", 1.2)
            self.CostFunction = OppositeSideMaxCostFunction("exponential",1.2,self.criticalDistance)
            # self.CostFunction = SegmentCostFunction_out0("exponential", 1.2)
            self.CostFunction.setSigma(100)
            self.PunchCost.initializeCostFunction(self.CostFunction)
            self.isInitializeCostDone = True
            self.SandbagPosition = self.PunchCost.getOriginalPoint()
            self.OptimalAction.initializeSandbagPosition(self.SandbagPosition)
        xAngleDiff = self.person_heading - 90
        self.CostFunction.updateSandbagPosition(self.SandbagPosition)
        isLeftClose, isRightClose = self.calculateIsclose()
        self.CostFunction.updateIsClose(isLeftClose, isRightClose)
        self.PunchCost.calculate_total_cost(self.humanPosition, self.center, self.leftPosition, self.rightPosition, xAngleDiff)
        
        # xMin, yMin = self.PunchCost.find_lowest_cost_point()
        # xOptimal, yOptimal = self.OptimalAction.getOptimalAction(self.leftPosition, self.rightPosition, self.center)
        
        self.SandbagPosition = self.OptimalAction.getOptimalAction_Graph(self.leftPosition, self.rightPosition, self.center, self.SandbagPosition)
        # self.OptimalAction.getOptimalAction_use_currentPosition()
        


        

    def calculateIsclose(self):
        leftDistance = np.linalg.norm([self.leftPosition[0] - self.center[0], self.leftPosition[1] - self.center[1]])        
        rightDistance = np.linalg.norm([self.rightPosition[0] - self.center[0], self.rightPosition[1] - self.center[1]])
        isLeftClose = leftDistance < self.criticalDistance
        isRightClose = rightDistance < self.criticalDistance
        return isLeftClose, isRightClose

    def drawGrid(self):
        self.drawCurrentPosition()
        angle = np.radians(self.person_heading - 90)
        
        colors = ["green", "red"]  # ì´ˆë¡ì—ì„œ ë¹¨ê°•ìœ¼ë¡œ
        cmap = LinearSegmentedColormap.from_list("cost_color_map", colors, N=256)
        norm = Normalize(vmin=0, vmax=2)  # ì½”ìŠ¤íŠ¸ ê°’ì˜ ì˜ˆìƒ ë²”ìœ„ ì„¤ì •
        scalar_map = ScalarMappable(norm=norm, cmap=cmap)
        currentX, currentY = self.SandbagPosition.getPosition()

        for index, (x, y) in enumerate(zip(self.PunchCost.xMesh, self.PunchCost.yMesh)):
            cost_value = self.PunchCost.C_total[index]
            rgba_color = scalar_map.to_rgba(cost_value)
            bgr_color = (rgba_color[2] * 255, rgba_color[1] * 255, rgba_color[0] * 255)  # RGBA to BGR and scaling to 0-255
            transformed_point = self.CoordinateTransformer.transform(self.center, angle, [x, y])
            
            # Check if it is the minimum cost point
            if x == currentX and y == currentY:
                # Draw a larger red circle for the minimum cost point
                self.draw_circle_on_frame(transformed_point, 10, (0, 255, 0), -1)  
                text_color = (0, 0, 255)  # Red color for text in BGR format
                font_size = 0.6  # Larger font size for minimum cost point
            else:
                # self.draw_circle_on_frame(transformed_point, 1, bgr_color, 2)
                text_color = (rgba_color[2] * 255, rgba_color[1] * 255, rgba_color[0] * 255)  # RGB to BGR for OpenCV
                font_size = 0.4

            font = cv2.FONT_HERSHEY_SIMPLEX
            text_position = (int(transformed_point[0] + 10), int(transformed_point[1]))  # í…ìŠ¤íŠ¸ ìœ„ì¹˜ ì¡°ì • (ì› ì˜†)
            cv2.putText(self.frame, f"{cost_value:.2f}", text_position, font, font_size, text_color, 1, cv2.LINE_AA)

    def drawCurrentPosition(self):
        angle = np.radians(self.person_heading - 90)
        currentX, currentY = self.SandbagPosition.getPosition()
        transformed_point = self.CoordinateTransformer.transform(self.center, angle, [currentX, currentY])
        self.draw_circle_on_frame(transformed_point, 10, (0, 255, 0), -1)


    def update_visualization(self):
        # Get positions from your tracking system
        human_position = (int(self.humanPosition[0]), int(self.humanPosition[0]))
        right_position = (int(self.rightPosition[0]), int(self.rightPosition[1]))
        left_position = (int(self.leftPosition[0]), int(self.leftPosition[1]))
        print(human_position)
        # Draw lines with gradients
        self.frame = self.draw_line_with_gradient(self.frame, human_position, right_position)
        self.frame = self.draw_line_with_gradient(self.frame, human_position, left_position)

        # Additional debug information
        cv2.imshow("Integrated System", self.frame)
        cv2.waitKey(1)



    def drawOnFrame(self):
        # self.showOptimalAction()
        # self.showPossibleAction()

        self.showIsPunch()
        # self.showDangerReason() # TODO tilt_2_0 update this method
        # self.showPunchVel()

        # self.putTextOnFrame(f"isLeftComingCLose: {self.isLeftComingClose}", (50,300),1, (255,0,0))
        # self.putTextOnFrame(f"isRightComingCLose: {self.isRightComingClose}", (50,330),1, (255,0,0))

        self.putTextOnFrame(f"right direction: {self.rightDirection: .2f}", (50,400),1, (100,100,200))
        self.putTextOnFrame(f"left direction: {self.leftDirection: .2f}", (50,430),1, (100,100,200))

        self.putTextOnFrame(f"person heading: {self.person_heading: .2f}", (50,600),1, (100,255,0))
        self.putTextOnFrame(f"right heading: {self.right_heading: .2f}", (50,630),1, (100,255,0))
        self.putTextOnFrame(f"left heading: {self.left_heading: .2f}", (50,660),1, (100,255,0))
        self.putTextOnFrame(f"marker heading: {self.markerHeading: .2f}", (50,690),1, (100,255,0))
        
        self.putTextOnFrame(f"fps: {self.fps}", (100,30),1, (255,0,0))

        self.draw_circle_on_frame(self.humanPosition,10,(0,255,0),2)
        self.draw_circle_on_frame(self.rightPosition,10,(0,255,0),2)
        self.draw_circle_on_frame(self.leftPosition,10,(0,255,0),2)
        self.draw_circle_on_frame(self.center,10,(0,0,255),2)

        self.draw_circle_on_frame(self.center,self.OptimalAction.getSemicriticalDistance(),(0,0,255),2)



        # self.showHandPositions('right')


        # self.showPunchType()
        # print(f"main.drawOnFrame() -> np.around(self.Tilt.getRadius()), is {int(np.around(self.Tilt.getRadius()))}")
        # self.draw_circle_on_frame(self.center, int(np.around(self.Tilt.getRadius())), (0,0,255), 2)
        # self.showHitPoint()

        # self.putTextOnFrame(f"is Left Punch End: {self.Tilt.isLeftPunchEnd()}", (700,270),1, (255,0,0) )
        # self.putTextOnFrame(f"is Right Punch End: {self.Tilt.isRightPunchEnd()}", (700,300),1, (255,0,0) )



    def punblish(self):
        # Float64MultiArray ë©”ì‹œì§€ ì¸ìŠ¤í„´ìŠ¤ ìƒì„±
        msg = Float32MultiArray()
        # optimal_action = self.SandbagPosition.getPosition()

        # axis0_degree, axis1_degree = self.ControlRotation.rotateAnyPoint(optimal_action[0], optimal_action[1], np.deg2rad(self.relative_heading))
        # axis0_position = self.ControlRotation.degreeToPose(axis0_degree)
        # axis1_position = self.ControlRotation.degreeToPose(axis1_degree)
        # print(f"axis0_position, axis1_position = {axis0_position:.2f}, {axis1_position:.2f}")
    
        msg.data = [float(self.circleX), float(self.circleY)]
        self.action_publisher.publish(msg)


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



    def showPunchVel(self):
        self.putTextOnFrame(f"left speed:{self.leftSpeed: .2f}",(50,170),1,(0,255,0))
        self.putTextOnFrame(f"left acc mag:{self.Tilt.Left.get_latest_acceleration(): .1f}, left acc dir:{self.Tilt.Left.get_latest_acc_direction(): .1f} ",(50,195),1,(0,255,0))
        self.putTextOnFrame(f"right speed:{self.rightSpeed: .2f}",(50,220),1,(0,255,0))
        self.putTextOnFrame(f"right acc mag:{self.Tilt.Right.get_latest_acceleration(): .1f}, left acc dir:{self.Tilt.Right.get_latest_acc_direction(): .1f} ",(50,245),1,(0,255,0))

    def showPunchType(self):
        Left, Right = self.Tilt.getPunch()
        self.putTextOnFrame(f"left Punch Type: {Left.getPunchType()}",(600,170),1.5,(0,255,0))
        self.putTextOnFrame(f"right Punch Type: {Right.getPunchType()}",(600,210),1.5,(0,255,0))


    def calculateDegree(self, action):
        ## person headingì—ì„œ ë°”ê¿€ê¹Œ????
        return np.deg2rad(self.person_heading) + action + math.pi / 2


    def showOptimalAction(self):
        
        if self.optimal_action != (None or 1000):
            optimalAbsolute = self.calculateDegree(self.optimal_action)
            optimalAbsolute
            sampleDistane = 100
            optimalCoordinate = [self.center[0] - sampleDistane * math.cos(optimalAbsolute), self.center[1] - sampleDistane * math.sin(optimalAbsolute)]
            cv2.circle(self.frame, (int(optimalCoordinate[0]), int(optimalCoordinate[1])), 30, (0, 0, 255), -1)
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
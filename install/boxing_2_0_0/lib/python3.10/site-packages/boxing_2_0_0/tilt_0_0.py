import math
import numpy as np


class Tilt():
    def __init__(self):

        ################ tune this #########################################
        self.criticalVelocity = 12
        self.criticalAngleDiff = 45
        self.criticalDistance = 200

        # self.posibleTilt = [0, -np.pi / 4, -np.pi / 2, -3 * np.pi / 4, -np.pi]
        self.posibleTilt = np.linspace(0,-np.pi,12)
        #####################################################################


        self.green_speed = 0
        self.blue_speed = 0

        self.green_coordinate = [0,0]
        self.blue_coordinate = [0,0]

        self.green_direction = 0
        self.blue_direction = 0

        self.center = [0,0]

        self.optimal_action = None

        self.velDanger = False
        self.distanceDanger = False
        self.greenDanger = False
        self.blueDanger = False

        self.person_heading = 0
        self.green_heading = 0
        self.blue_heading = 0
        self.punch_heading = 0
        

    def initializeLeft(self, coordinate, speed, direction):
        self.green_coordinate = coordinate
        self.green_speed = speed
        self.green_direction = direction

    def initializeRight(self, coordinate, speed, direction):
        self.blue_coordinate = coordinate
        self.blue_speed = speed
        self.blue_direction = direction

    def initializePersonHeading(self, personHeading, leftHeading, rightHeading):
        self.person_heading = personHeading
        self.green_heading = leftHeading
        self.blue_heading = rightHeading

    def initializeCenter(self,center):
        self.center = center


    def detectPunch(self):
        # Handle cases where speeds might not be set
        self.velocity = None
        if self.green_speed is not None and self.blue_speed is not None:
            self.velocity = max(self.green_speed, self.blue_speed)
        elif self.green_speed is not None:
            self.velocity = self.green_speed
        elif self.blue_speed is not None:
            self.velocity = self.blue_speed

        # Ensure coordinates are initialized before calculating distances
        greenDistance = self.calculate_distance(self.green_coordinate, self.center) if self.green_coordinate is not None else float('inf')
        blueDistance = self.calculate_distance(self.blue_coordinate, self.center) if self.blue_coordinate is not None else float('inf')

        # Determine punch direction and distance based on which speed is greater
        if self.velocity == self.green_speed:
            self.punchDirection = self.green_direction
            self.punch_heading = self.green_heading
            self.distance = greenDistance

        elif self.velocity == self.blue_speed:
            self.punchDirection = self.blue_direction
            self.punch_heading = self.blue_heading
            self.distance = blueDistance

        # print(self.punchDirection)

        # Calculate the absolute angle difference
        angleDiff = abs(self.punchDirection - self.punch_heading )if self.punchDirection is not None and self.punch_heading is not None else None 
        ## 여기를 person heading이 아니라  punch heading으로 구하기

        # Decide action based on velocity and angle difference
        if self.velocity is not None and self.velocity > self.criticalVelocity:
            if angleDiff is not None and angleDiff < self.criticalAngleDiff:
                self.optimal_action = self.tilt()

                self.setDanger(True,False)
            else:
                self.optimal_action = None

                self.setDanger(False,False)
        else:
            if greenDistance < self.criticalDistance:
                self.punchDirection = self.green_direction
                self.distance = greenDistance
                self.punch_heading = self.green_heading
                self.optimal_action = self.tilt()

                self.setDanger(False,True)

            elif blueDistance < self.criticalDistance:
                self.punchDirection = self.blue_direction
                self.distance = blueDistance
                self.punch_heading = self.blue_heading
                self.optimal_action = self.tilt()

                self.setDanger(False,True)
            else:
                self.optimal_action = None

                self.setDanger(False,False)


        return self.optimal_action
    
    def setDanger(self, velDanger, distDanger):
        self.velDanger = velDanger
        self.distanceDanger = distDanger

    def setPunch(self,green,blue):
        self.greenDanger = green
        self.blueDanger = blue
    
    def isVelDanger(self):
        return self.velDanger
    
    def isDistanceDanger(self):
        return self.distanceDanger


    def calculateAngleDifference(self, action):
        # person_heading_scalar = float(self.person_heading)
        # action_scalar = float(action)
        # return person_heading_scalar + action_scalar + math.pi / 2
        # print(f"action is {action}")
        # print(f"self.person_heading is {self.person_heading}")
        absoluteActionDegree =  np.rad2deg(np.deg2rad(self.person_heading) + action + math.pi / 2)
        
        punchRightAvoidDegree = (self.punch_heading - 90) if (self.punch_heading - 90) > - 180 else (self.punch_heading - 90) + 360
        punchLeftAvoidDegree  = (self.punch_heading + 90) if (self.punch_heading + 90) < 180 else (self.punch_heading + 90) - 360
    
        rightDifference = self.getAbsoluteAngleDiff(absoluteActionDegree, punchRightAvoidDegree)
        leftDifference  = self.getAbsoluteAngleDiff(absoluteActionDegree, punchLeftAvoidDegree)

        return min(rightDifference, leftDifference)

    def getAbsoluteAngleDiff(self, angle1, angle2):
        diff = abs(angle1 - angle2)
        if diff > 180:
            diff = abs(diff - 360)

        return diff

    def calculate_distance(self, color_coordinate, frame_center):
        # Calculate Euclidean distance between the color coordinate and the frame center
        return math.sqrt((color_coordinate[0] - frame_center[0])**2 + (color_coordinate[1] - frame_center[1])**2)


    def tilt(self):
        angleDiff = {}
        for action in self.posibleTilt:
            angleDiff[action] = self.calculateAngleDifference(action)

        optimal_action = min(angleDiff, key=angleDiff.get) #orig = max
        # Command to control the robot's tilting mechanism
        # Assuming there's a method or ROS topic to handle this
        # self.ros_tilt_publisher.publish(optimal_action)
        return optimal_action


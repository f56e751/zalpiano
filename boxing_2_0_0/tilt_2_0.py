import math
import numpy as np
from logPunch import PunchData
from punch import Punch
from punchTypeDetector import PunchTypeDetector
import time

######### ver 0.1 ##############
# simplify code with making Punch() $ ComparePunch()

######### ver 1.1 ##############
# freeze position when punch is close or over center

######### ver 2.0 ##############
# predict human punch target point 
# detect punch Straight or Hook
# predict punch point


class Tilt():
    def __init__(self):
        # distance variable has to be propotoinal to aruco size
        
        ################ tune this #########################################
        self.criticalVelocity = 12
        self.criticalAngleDiff = 45
        self.criticalDistance = 200

        self.closeDistance = 150

        # self.posibleTilt = [0, -np.pi / 4, -np.pi / 2, -3 * np.pi / 4, -np.pi]
        self.posibleTilt = np.linspace(0,-np.pi,12)
        #####################################################################

        self.center = [0,0]

        self.person_heading = 0
        self.personPosition = [0,0]

        self.Left = Punch()
        self.Right = Punch()
        self.Left.setCriticalAngleDiff(self.criticalAngleDiff)
        self.Right.setCriticalAngleDiff(self.criticalAngleDiff)

        # self.ComparePunch = ComparePunch(self.Left, self.Right)
        # self.ComparePunch.setCriticalValue(self.criticalVelocity,self.criticalDistance)
        self.PunchTypeDetector = PunchTypeDetector(self.Left,self.Right)

        self.prePunchHeadDistance = 0

        self.count = 0

        self.DangerPunch = None
        self.optimalAction = None
        self.preOptimalAction = None

    def initializeLeft(self, coordinate, speed, direction, heading):
        self.Left.add_data(speed, direction, time.time())
        self.Left.initialize(coordinate, speed, direction, heading)

    def initializeRight(self, coordinate, speed, direction, heading):
        self.Right.add_data(speed, direction, time.time())
        self.Right.initialize(coordinate, speed, direction, heading)

    def initializePersonHeading(self, personHeading):
        self.person_heading = personHeading

    def initializePersonPosition(self, personPosition):
        self.personPosition = personPosition

    def initializeCenter(self,center):
        self.center = center
    
    def detectPunch(self):
        self.ComparePunch.setDistance(self.center)
        optimalAction = None

        if self.ComparePunch.isVelDanger(self.criticalVelocity):
            
            self.DangerPunch = self.ComparePunch.getDangerPunch()
            optimalAction = self.tilt()
        elif self.ComparePunch.isDistanceDanger(self.criticalDistance):
            
            self.DangerPunch = self.ComparePunch.getDangerPunch()
            optimalAction = self.tilt()

        if (self.isPunchClose() and self.isActionJump()) or self.isPunchOverCenter():
            # print(f"actionJump! {self.count} times, optimal action is {self.optimalAction}, preoptimalaction is {self.preOptimalAction}")
            print(f"freeze movement {self.count} times")
            self.count += 1
            # when punch is near center and optimal action changes drastically, optimal action doesn't change!!
            optimalAction = self.preOptimalAction

        # if self.isPunchEnd():
        #     print("punch End")
        #     optimalAction = self.preOptimalAction

        if optimalAction is not None:
            self.preOptimalAction = optimalAction
        return optimalAction
    

    def isPunchOverCenter(self):
        if self.DangerPunch is not None:
            punchPersonDistance = self.getDistance(self.DangerPunch.coordinate, self.personPosition) 
            centerPersonDistance = self.getDistance(self.personPosition, self.center)
            return punchPersonDistance > centerPersonDistance * 1.2
        else:
            return False
    def isActionJump(self):
        
        if self.optimalAction is not None and self.preOptimalAction is not None:
            return abs(self.optimalAction - self.preOptimalAction) > np.pi * 0.6
    
    def isVelDanger(self):
        return self.ComparePunch.isVelDanger(self.criticalVelocity)
    
    def isDistanceDanger(self):
        return self.ComparePunch.isDistanceDanger(self.criticalDistance)
    
    def isPunchClose(self):
        if self.DangerPunch is not None:
            return self.DangerPunch.distance < self.closeDistance
        else:
            return False
        
    # def isColliding(self):
        

    def isPunchEnd(self):
        if self.DangerPunch is not None:
            punchHeadDistance = self.getDistance(self.DangerPunch.coordinate, self.personPosition)
            if punchHeadDistance < self.prePunchHeadDistance:
                isPunchEnd = True
            else:
                isPunchEnd = False

            self.prePunchHeadDistance = punchHeadDistance
            return isPunchEnd
        else:
            return False

    def getDistance(self, position1, position2):
        return math.sqrt((position1[0] - position2[0])**2 + (position1[1] - position2[1])**2)




    def calculateAngleDifference(self, action):
        absoluteActionDegree =  np.rad2deg(np.deg2rad(self.person_heading) + action + math.pi / 2)
        punchHeading = self.DangerPunch.heading

        punchRightAvoidDegree = (punchHeading - 90) if (punchHeading - 90) > - 180 else (punchHeading - 90) + 360
        punchLeftAvoidDegree  = (punchHeading + 90) if (punchHeading + 90) < 180 else (punchHeading + 90) - 360
    
        rightDifference = self.getAbsoluteAngleDiff(absoluteActionDegree, punchRightAvoidDegree)
        leftDifference  = self.getAbsoluteAngleDiff(absoluteActionDegree, punchLeftAvoidDegree)

        return min(rightDifference, leftDifference)

    def getAbsoluteAngleDiff(self, angle1, angle2):
        diff = abs(angle1 - angle2)
        if diff > 180:
            diff = abs(diff - 360)

        return diff

    def tilt(self):
        angleDiff = {}
        for action in self.posibleTilt:
            angleDiff[action] = self.calculateAngleDifference(action)

        self.optimalAction =  min(angleDiff, key=angleDiff.get)
        return self.optimalAction

    

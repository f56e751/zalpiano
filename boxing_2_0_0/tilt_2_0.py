import math
import numpy as np

import sys
import os

# boxing_2_0_0 디렉토리의 부모 디렉토리를 sys.path에 추가
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from boxing_2_0_0 import Punch, PunchTypeDetector, PunchPriority, TiltPosition

# from .punch import Punch
# from .punchTypeDetector import PunchTypeDetector
# from .tiltposition import TiltPosition
# from .punchPriority import PunchPriority

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
        # 아루코마커 크기에 비례한 값으로 수정하기
        self.criticalVelocity = 12
        self.criticalAngleDiff = 45
        self.criticalDistance = 200
        self.hitPointDiff = 15

        self.closeDistance = 150

        # self.posibleTilt = [0, -np.pi / 4, -np.pi / 2, -3 * np.pi / 4, -np.pi]
        self.posibleTilt = np.linspace(0,-np.pi,12)

        self.radius = 100
        self.pipeLength = 300  # length of carbon Pipe
        #####################################################################

        self.center = [0,0]
        
        self.Tiltposition = TiltPosition()
        # TODO Tiltposition 업데이트하기

        self.person_heading = 0
        self.personPosition = [0,0]

        self.Left = Punch("Left")
        self.Right = Punch("Right")
        self.Left.setCriticalAngleDiff(self.criticalAngleDiff)
        self.Right.setCriticalAngleDiff(self.criticalAngleDiff)

        self.PunchPriority = PunchPriority(self.Left, self.Right)

        # self.ComparePunch = ComparePunch(self.Left, self.Right)
        # self.ComparePunch.setCriticalValue(self.criticalVelocity,self.criticalDistance)
        self.PunchTypeDetector = PunchTypeDetector(self.Left, self.Right, self.Tiltposition)
        self.PunchTypeDetector.initializeCriticalValue(self.criticalVelocity, self.criticalDistance, self.hitPointDiff)

        self.prePunchHeadDistance = 0

        self.count = 0

        self.DangerPunch = None
        self.optimalAction = None
        self.preOptimalAction = None

    def initializeLeft(self, coordinate, speed, direction, heading, vel, currentTime):
        # TODO 나중에 하나로 통합하기
        self.Left.add_data(vel, speed, direction, currentTime)
        self.Left.initialize(coordinate, speed, direction, heading)

    def initializeRight(self, coordinate, speed, direction, heading, vel, currentTime):
        self.Right.add_data(vel, speed, direction, currentTime)
        self.Right.initialize(coordinate, speed, direction, heading)

    def initializePersonHeading(self, personHeading):
        self.person_heading = personHeading

    def initializePersonPosition(self, personPosition):
        self.personPosition = personPosition

    def initializeCenter(self,center):
        self.center = center

    # def initializePipeLength(self, pipeLength):
    #     self.pipeLength = pipeLength

    def setPunchTypeDetector(self):
        # self.PunchTypeDetector.setRadius(self.pipeLength * np.cos(np.deg2rad(self.Tiltposition.getTiltAngle())))
        # TODO add current position to radius
        self.radius = self.pipeLength * np.sin(np.deg2rad(20))
        self.PunchTypeDetector.setRadius(self.radius)
        self.PunchTypeDetector.setPerson(self.person_heading, self.personPosition)
        self.PunchTypeDetector.setCenter(self.center)


    def detectPunch(self):
        self.Left.setDistance(self.center)
        self.Right.setDistance(self.center)
        self.setPunchTypeDetector()

        self.leftHitPoint = None
        self.rightHitPoint = None
        # print("self.pipeLength is: ", self.pipeLength)

        hitPointRanges = {
            "Left": None,
            "Right": None
        }

        for hand in ["Left", "Right"]:
            self.PunchTypeDetector.detectPunchType(getattr(self, hand))
            if getattr(self, hand).isDanger():
                hitPointRanges[hand] = self.PunchTypeDetector.getHitPointRange(getattr(self, hand))
                # print(f"Tilt.detectPunch() -> hitPointRanges[hand] is {hitPointRanges[hand]}")
        leftHitPointRange = hitPointRanges["Left"]
        rightHitPointRange = hitPointRanges["Right"]



        if leftHitPointRange and rightHitPointRange:
            if self.PunchPriority.getPriorityPunch().isLeft():
                optimalAction = leftHitPointRange["rightBoundary"]
            else:
                optimalAction = rightHitPointRange["leftBoundary"]
        elif leftHitPointRange:
            optimalAction = leftHitPointRange["rightBoundary"]
        elif rightHitPointRange:
            optimalAction = rightHitPointRange["leftBoundary"]
        else:
            optimalAction = None

        if optimalAction is not None:
            # print(f"class Tilt() optimal action is {optimalAction}")
            if math.isnan(optimalAction):
                print("optimal action is nan")

        return optimalAction
    
    def getHitPointUnitTest(self):
        self.Left.setDistance(self.center)
        self.Right.setDistance(self.center)
        self.setPunchTypeDetector()

        leftHitPoint = self.PunchTypeDetector.getHitPointUnitTest(self.Left)
        rightHitPoint = self.PunchTypeDetector.getHitPointUnitTest(self.Right)
        return leftHitPoint, rightHitPoint
    
    def returnHitPoint(self):
        return self.PunchTypeDetector.returnHitPoint()
    
    def isLeftPunchEnd(self):
        return self.PunchTypeDetector.isPunchEnd(self.Left)
    
    def isRightPunchEnd(self):
        return self.PunchTypeDetector.isPunchEnd(self.Right)

    
    def getPunch(self):
        return self.Left, self.Right
    
    def getRadius(self):
        return self.radius
   
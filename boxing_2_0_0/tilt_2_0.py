import math
import numpy as np
from logPunch import PunchData
from punch import Punch
from punchTypeDetector import PunchTypeDetector
from tiltposition import TiltPosition
from punchPriority import PunchPriority
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
        self.hitPointDiff = np.deg2rad(15)

        self.closeDistance = 150

        # self.posibleTilt = [0, -np.pi / 4, -np.pi / 2, -3 * np.pi / 4, -np.pi]
        self.posibleTilt = np.linspace(0,-np.pi,12)
        #####################################################################

        self.center = [0,0]
        self.pipeLength = None  # length of carbon Pipe
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

    def initializePipeLength(self, pipeLength):
        self.pipeLength = pipeLength

    def setPunchTypeDetertor(self):
        self.PunchTypeDetector.setRadius(self.pipeLength * np.cos(np.deg2rad(self.Tiltposition.getTiltAngle)))
        self.PunchTypeDetector.setPerson(self.person_heading, self.personPosition)
        self.PunchTypeDetector.setCenter(self.center)


    def detectPunch(self):
        self.setPunchTypeDetertor()
        self.PunchTypeDetector.detectPunchType(self.Left)
        self.PunchTypeDetector.detectPunchType(self.Right)
        leftHitPointRange = None
        rightHitPointRange = None

        if self.Left.isDanger():
            leftHitPointRange = self.PunchTypeDetector.getHitPointRange(self.Left)
        elif self.Right.isDanger():
            rightHitPointRange = self.PunchTypeDetector.getHitPointRange(self.Right)


        if leftHitPointRange != None and rightHitPointRange != None:
            # 둘 다 있을 때는 우선순위를 정해서 피함 (아니면 범위 밖으로 피해도 될듯)
            if self.PunchPriority().getPriorityPunch().isLeft():
                optimalAction = leftHitPointRange["leftBoundary"] 
            else:
                optimalAction = rightHitPointRange["rightBoundary"]

        if leftHitPointRange != None:
            optimalAction = leftHitPointRange["leftBoundary"] 
        elif rightHitPointRange != None:
            optimalAction = rightHitPointRange["rightBoundary"]
        else:
            # 펀치가 없을 때
            optimalAction = None
            # optimalAction = self.preOptimalAction

        # self.preOptimalAction = optimalAction

        return optimalAction
   
import sys
import os

# boxing_2_0_0 디렉토리의 부모 디렉토리를 sys.path에 추가
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from boxing_2_0_0 import PunchDataNumpy
import math

class Punch(PunchDataNumpy):
    def __init__(self, hand):
        super().__init__()

        self.coordinate = [0,0]
        self.speed = 0
        self.direction = 0
        self.heading = 0

        self.criticalAngleDiff = None

        # self.isDanger = False
        self.velDanger = False
        self.distanceDanger = False

        self.distance = 0
        self.punchTypes = ["None", "Straight", "Hook", "SlowButClose", "PunchEnd"]
        #  속도는 충족하는데 ispunchEnd()로 방향이 다른거는 PunchEnd로 변경

        if hand == 'Left':
            self.Hand = 'Left'
        elif hand == 'Right':
            self.Hand = 'Right'
        else:
            ValueError("hand has to be Left or Right")
        
        
        self.punchType = "None"
        

    def initialize(self, coordinate, speed, direction, heading):
        self.coordinate = coordinate
        self.speed = speed
        self.direction = direction
        self.heading = heading
    
    def isLeft(self):
        if self.Hand == None:
            ValueError("self.Hand is not initialized")
        return self.Hand == 'Left'


    def setCriticalAngleDiff(self, criticalAngleDiff):
        self.criticalAngleDiff = criticalAngleDiff

    def isInAngle(self):
        return abs(self.direction - self.heading) < self.criticalAngleDiff

    def setDistance(self, centerCoordinate):
        self.distance =  math.sqrt((self.coordinate[0] - centerCoordinate[0])**2 + (self.coordinate[1] - centerCoordinate[1])**2)

    def setPunchType(self, punchType):
        if punchType in self.punchTypes:
            self.punchType = punchType
        else:
            raise ValueError("punchType not in punchTypes")
        

    def isDanger(self):
        return self.punchType != "None"
    
    def getPunchType(self):
        return self.punchType
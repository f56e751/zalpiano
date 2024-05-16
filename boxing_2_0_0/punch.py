from logPunch import PunchData
import math

class Punch(PunchData):
    def __init__(self):
        super().__init__()
        self.coordinate = [0,0]
        self.speed = 0
        self.direction = 0
        self.heading = 0

        self.criticalAngleDiff = None

        self.isDanger = False
        self.velDanger = False
        self.distanceDanger = False

        self.distance = 0
        self.punchTypes = ["None", "Straight", "Hook", "SlowButClose"] 
        self.punchType = None

    def initialize(self, coordinate, speed, direction, heading):
        self.coordinate = coordinate
        self.speed = speed
        self.direction = direction
        self.heading = heading

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
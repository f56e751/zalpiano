from punch import Punch
import numpy as np
import math

class PunchTypeDetector():
    def __init__(self, Left: Punch, Right: Punch):
        # self.ComparePunch = ComparePunch(Left, Right)
        self.Left = Left
        self.Right = Right
        # self.DangerPunch = None
        self.compareSpeed = None
        self.radius = 0
        self.personHeading = 0
        self.personCoordinate = 0
        self.center = [0,0]
        self.hitPointDiff = np.deg2rad(15)

    def setRadius(self,r):
        self.radius = r

    def setCriticalValue(self, vel, distance):
        self.criticalVel = vel
        self.criticalDistance = distance

    def setPerson(self, personHeading, personCoordinate):
        self.personHeading = personHeading
        self.personCoordinate = personCoordinate
    
    def setCenter(self, center):
        self.center = center

    def setHitPointDiff(self, hitPointDiff):
        # hitPointDiff means angle boundary size for hitPoint
        self.hitPointDiff = hitPointDiff

    def detectPunchType(self, Punch: Punch):
        # 왼손과 오른손 둘 다 펀치 type 설정
        # 둘 다 펀치로 인식되면 나중에 우선순위를 정해서 회피하기 -> Tilt 클래스에서 수행
        if self.criticalVel is None or self.criticalDistance is None:
            raise ValueError("critical value has not been set.")
        
        diff = self.getAbsoluteAngleDiff(Punch.direction, self.personHeading)
        if Punch.speed > self.criticalVel:
            if diff < 25:
                Punch.setPunchType("Straight")

            elif self.isHook(Punch):
                Punch.setPunchType("Hook")

            else:
                Punch.setPunchType("None")
        else:
            if Punch.distance < self.criticalDistance:
                Punch.setPunchType("SlowButClose")
                
            else:
                Punch.setPunchType("None")


    # def isHook(self, Punch: Punch): 
    # TODO         

    def getHitPointRange(self, Punch: Punch):
        punchType = Punch.punchType
        hitPoint = 0
        if punchType is "Straight":
            hitPoint = self.getHitPoint(Punch.direction)
            leftDirection = Punch.direction - self.hitPointDiff
            rightDirection = Punch.direction + self.hitPointDiff
            leftBoundary = self.getHitPoint(leftDirection)
            rightBoundary = self.getHitPoint(rightDirection)
            return [leftBoundary, rightBoundary]
            
        # elif punchType is "Hook":
        # TODO



        return hitPoint


    def isDirectionLeftToCenter(self, Punch: Punch, direction):
        diff = Punch.direction - Punch.heading
        diff = direction - self.personHeading
        if diff > 180:
            diff = diff - 360
        
        if diff < 0:
            return True
        else:
            return False
        
    def getHitPoint(self, Punch: Punch, direction): # alpha means degree between point(line and half circle intersection point) and half circle line
        dx, dy = math.cos(direction), math.sin(direction)
        x1, y1 = Punch.coordinate[0], Punch.coordinate[1]
        a, b = self.center[0], self.center[1]
        lineAndCenterDistance = abs(dy * a - dx * b - (x1 * dy - y1 * dx)) / (dx**2 + dy**2)**(1/2)
        centerAndHumanDistance = ((self.center[0] - self.personCoordinate[0])**2 + (self.center[1] - self.personCoordinate[1])**2)**(1/2)
        theta = np.arcsin(lineAndCenterDistance / centerAndHumanDistance) # rad
        alpha = np.arccos(lineAndCenterDistance / self.radius) - theta    # rad

        if self.isDirectionLeftToCenter(direction):
            hitPoint = - np.pi + alpha
        else:
            hitPoint = alpha
        return hitPoint



    def getAbsoluteAngleDiff(self, angle1, angle2):
        diff = abs(angle1 - angle2)
        if diff > 180:
            diff = abs(diff - 360)

        return diff
import math
import numpy as np

######### ver 0.1 ##############
# simplify code with making Punch() $ ComparePunch()

######### ver 1.1 ##############
# freeze position when punch is close or over center


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

        self.ComparePunch = ComparePunch(self.Left, self.Right)

        self.prePunchHeadDistance = 0

        self.count = 0

        self.DangerPunch = None
        self.optimalAction = None
        self.preOptimalAction = None

    def initializeLeft(self, coordinate, speed, direction, heading):
        self.Left.initialize(coordinate, speed, direction, heading)

    def initializeRight(self, coordinate, speed, direction, heading):
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

    

class Punch():
    def __init__(self):
        self.coordinate = [0,0]
        self.speed = 0
        self.direction = 0
        self.heading = 0

        self.criticalAngleDiff = None

        self.isDanger = False
        self.velDanger = False
        self.distanceDanger = False

        self.distance = 0

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


class ComparePunch():
    ### None인지 고려하기!!!!!!!!!!!!!!!
    def __init__(self, Left: Punch, Right: Punch):
        self.Left = Left
        self.Right = Right
        self.DangerPunch = None

    def setDistance(self, centerCoordinate):
        self.DangerPunch = None
        self.Left.setDistance(centerCoordinate)
        self.Right.setDistance(centerCoordinate)

    def getDangerPunch(self):
        return self.DangerPunch

    def getHigherVel(self):
        if self.Left.speed > self.Right.speed:
            return self.Left
        else:
            return self.Right
        
    def getCloseDistance(self):
        if self.Left.distance < self.Right.distance:
            return self.Left
        else:
            return self.Right
    
    def isVelDanger(self, criticalVel: float) -> bool:
        higherVelPunch = self.getHigherVel()
        isDanger = higherVelPunch.speed > criticalVel and higherVelPunch.isInAngle()
        if isDanger:
            self.DangerPunch = higherVelPunch
        return isDanger

    def isDistanceDanger(self, criticalDistance: float) -> bool:
        closeDistancePunch = self.getCloseDistance()
        isDanger = closeDistancePunch.distance < criticalDistance
        if isDanger:
            self.DangerPunch = closeDistancePunch
        return isDanger



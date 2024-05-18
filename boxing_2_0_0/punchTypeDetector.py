from punch import Punch
import numpy as np
import math
from tiltposition import TiltPosition

class PunchTypeDetector():
    def __init__(self, Left: Punch, Right: Punch, TiltPosition: TiltPosition):
        self.TiltPosition = TiltPosition
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

        ## hook recogition parameter ####
        self.criticalHookAcc = 10 # unit/s^2 -> 나중에 아로코 크기에 비례하도록 변경
        self.criticalHookDirectionChange = 20 # degree
        self.criticalHookDirectionChange_END = 10 # if self.getAbsoluteAngleDiff(accDirection, Punch.direction) is smaller than this value, this class judges that HOOK Circle trajectory is end
        self.hookRecognitionTime = 0.1  # sec

        self.leftHitPoint = None
        self.rightHitPoint = None

    def setRadius(self,r):
        if r is None:
            raise ValueError("radius has not been set.")
        self.radius = r
        

    def initializeCriticalValue(self, vel, distance, hitPointDiff):
        self.criticalVel = vel
        self.criticalDistance = distance
        self.hitPointDiff = hitPointDiff
        # self.radius = r

    def setPerson(self, personHeading, personCoordinate):
        self.personHeading = personHeading
        self.personCoordinate = personCoordinate
    
    def setCenter(self, center):
        self.center = center


        
    # def setCurrentTiltPosition(self, xyAngle, tiltAngle):
    #     # xy Angle means angle in xy plane, tiltANgle means angle between z axis and tilt
    #     self.TiltPosition.setPosition(xyAngle, tiltAngle)
    # tilt.py에서 TiltPosition 클래스 값을 업데이트해서 이 함수는 필요없음


    def detectPunchType(self, Punch: Punch):
        # 왼손과 오른손 둘 다 펀치 type 설정
        # 둘 다 펀치로 인식되면 나중에 우선순위를 정해서 회피하기 -> Tilt 클래스에서 수행
        if self.criticalVel is None or self.criticalDistance is None:
            raise ValueError("critical value has not been set.")
        
        # diff = self.getAbsoluteAngleDiff(Punch.direction, self.personHeading)
        diff = self.getAbsoluteAngleDiff(Punch.direction, Punch.get_latest_acc_direction())


        # TODO 이거 사용할지 결정하기
        # 펀치가 끝나지 않았다고 판단되면 그 펀치 종류에 대한 기존의 판단 유지 

        # if Punch.punchType == "Straight":
        #     if not self.isPunchEnd(Punch):
        #         return
        # elif Punch.punchType == "Hook":
        #     if not self.isPunchEnd(Punch):
        #         return


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
                # print(f"Punch.distance is {Punch.distance}") 

            else:
                Punch.setPunchType("None")


    def isHook(self, Punch: Punch):  
        # TODO
        # np.mean(accMagnitudes) < np.mean(accMagnitudes[-4:-1] 이 부분에 대한 튜닝 필요
        data = Punch.get_data_last_seconds(self.hookRecognitionTime)
        accMagnitudes = data['accelerations']
        accDirections = data ['accDirections']
        isAccIncreasing = np.mean(accMagnitudes[-1]) > self.criticalHookAcc and np.mean(accMagnitudes) < np.mean(accMagnitudes[-4:-1])
        isAccDirChanging = self.getAbsoluteAngleDiff(accDirections[-1], Punch.direction) > self.criticalHookDirectionChange
        if isAccIncreasing and isAccDirChanging:
            return True
        else:
            return False

    def isPunchEnd(self, Punch: Punch):
        return self.getAbsoluteAngleDiff(self.personHeading,Punch.direction) > 90
        
        
    def isHookCircleEnd(self, Punch: Punch):
        # if punch type is Hook, this function judges that circle trajectory part is End
        accDirection = Punch.get_latest_acc_direction()
        isAccDirChanging = self.getAbsoluteAngleDiff(accDirection, Punch.direction) < self.criticalHookDirectionChange_END
        return isAccDirChanging

    def recordHitPoint(self, Punch: Punch, hitPoint):
        if Punch.Hand == "Right":
            self.rightHitPoint = hitPoint
        elif Punch.Hand == "Left":
            self.leftHitPoint = hitPoint

    def returnHitPoint(self):
        return [self.leftHitPoint, self.rightHitPoint]

    def getHitPointRange(self, Punch: Punch):
        punchType = Punch.punchType
        hitPoint = 0
        if punchType == "Straight":
            hitPoint = self.getHitPoint(Punch, Punch.direction)
            self.recordHitPoint(Punch, hitPoint)
            leftDirection = Punch.direction - self.hitPointDiff
            rightDirection = Punch.direction + self.hitPointDiff
            leftBoundary = self.getHitPoint(Punch, leftDirection)
            rightBoundary = self.getHitPoint(Punch, rightDirection)
            return {"leftBoundary" : leftBoundary, "rightBoundary" : rightBoundary} 
            
        elif punchType == "Hook":
            if self.isHookCircleEnd(Punch):
                hitPoint = self.getHitPoint(Punch, Punch.direction)
                self.recordHitPoint(Punch, hitPoint)
                leftDirection = Punch.direction - self.hitPointDiff
                rightDirection = Punch.direction + self.hitPointDiff
                leftBoundary = self.getHitPoint(Punch, leftDirection)
                rightBoundary = self.getHitPoint(Punch, rightDirection)
                return {"leftBoundary" : leftBoundary, "rightBoundary" : rightBoundary} 

            else:
                # assume person is hitting currentPosition
                hitPoint = self.TiltPosition.getXYAngle()
                self.recordHitPoint(Punch, hitPoint)
                leftBoundary = hitPoint - self.hitPointDiff
                rightBoundary = hitPoint + self.hitPointDiff
                
                return {"leftBoundary" : leftBoundary, "rightBoundary" : rightBoundary}

        else:
            return None

    def getHitPointUnitTest(self,Punch: Punch):
        # hitPoint = self.getHitPoint(Punch, Punch.direction)
        hitPoint = self.getHitPoint(Punch, -90)
        return hitPoint

    def isDirectionLeftToCenter(self, Punch: Punch, direction):
        diff = direction - Punch.heading
        if diff > 180:
            diff = diff - 360
        elif diff < -180:
            diff = diff + 360
        
        if diff < 0:
            return True
        else:
            return False
        
    # def getHitPoint(self, Punch: Punch, direction): # alpha means degree between point(line and half circle intersection point) and half circle line
    #     dx, dy = math.cos(direction), math.sin(direction)
    #     x1, y1 = Punch.coordinate[0], Punch.coordinate[1]
    #     a, b = self.center[0], self.center[1]
    #     lineAndCenterDistance = abs(dy * a - dx * b - (x1 * dy - y1 * dx)) / (dx**2 + dy**2)**(1/2)
    #     centerAndHumanDistance = ((self.center[0] - self.personCoordinate[0])**2 + (self.center[1] - self.personCoordinate[1])**2)**(1/2)
    #     theta = np.arcsin(lineAndCenterDistance / centerAndHumanDistance) # rad
    #     print(f"punchTypeDetector -> lineAndCenterDistance: {lineAndCenterDistance}")
    #     print(f"punchTypeDetector -> self.radius: {self.radius}")
    #     alpha = np.arccos(lineAndCenterDistance / self.radius) - theta    # rad
    #     print("alpha is: ", alpha)
    #     if self.isDirectionLeftToCenter(Punch, direction):
    #         hitPoint = - np.pi + alpha
    #     else:
    #         hitPoint = alpha
    #     if math.isnan(hitPoint):
    #         print("hitpoint is Nan")
    #     print(f"punchTypeDetector -> hitpoint is: {hitPoint}")
    #     return hitPoint

    def getHitPoint(self, Punch: Punch, direction):
        v1, v2 = np.cos(np.deg2rad(direction)), np.sin(np.deg2rad(direction))
        x1, y1 = Punch.coordinate[0], Punch.coordinate[1]
        a, b = self.center[0], self.center[1]
        # print([(v1**2 + v2**2), (v1 * (x1 - a) + v2 * (y1 - b), ((x1 - a)**2 + (y1 - b)**2 - self.radius**2))])
        roots = np.roots([v1**2 + v2**2, 2 * v1 * (x1 - a) + 2 * v2 * (y1 - b), ((x1 - a)**2 + (y1 - b)**2 - self.radius**2)])
        # x = x1 + v1 * t, y = y1 + v2 * t
        # root means t
        personToCenterVector = np.array([a - self.personCoordinate[0], b - self.personCoordinate[1]])
        baselineVector = self.rotate90degree(personToCenterVector)
        if np.iscomplex(roots[0]):
            hitpoint = self.getOutRangeHitPoint(Punch, direction)
        elif roots[0] == roots[1]:
            x = x1 + v1 * roots[0]
            y = y1 + v2 * roots[0]
            centerToPointVector = np.array([x - a, y - b])
            if np.dot(centerToPointVector, personToCenterVector) <= 0:
                hitpoint = self.getOutRangeHitPoint(Punch, direction)
            else:
                hitpoint = - self.angle_between_vectors(baselineVector, centerToPointVector)
        else:
            x_a = x1 + v1 * roots[0]
            y_a = y1 + v2 * roots[0]

            x_b = x1 + v1 * roots[0]
            y_b = y1 + v2 * roots[0]

            if np.dot(np.array([x_a - a, y_a - b]), personToCenterVector) > 0:
                centerToPointVector = np.array([x_a - a, y_a - b])
            else:
                centerToPointVector = np.array([x_b - a, y_b - b])

            hitpoint = - self.angle_between_vectors(baselineVector, centerToPointVector)
        print(f"class punchTypeDetector.getHitPoint()-> np.iscomplex(roots[0]): {np.iscomplex(roots[0])}")
        print(f"class punchTypeDetector.getHitPoint()-> hitpoint is: {hitpoint}, punchType is {Punch.getPunchType()}, speed is {Punch.speed}")
        return hitpoint
    
    def getOutRangeHitPoint(self, Punch, direction):
        if self.isDirectionLeftToCenter(Punch, direction):
            return - np.pi
        else:
            return 0
        
    def rotate90degree(self,vector):
        rotation = np.array(([0,1],[-1,0]))
        return np.dot(vector, rotation)
    

    def angle_between_vectors(self, v1, v2):
        # Calculate the dot product
        dot_product = np.dot(v1, v2)
        
        # Calculate the magnitudes (norms) of the vectors
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        
        # Calculate the cosine of the angle
        cos_angle = dot_product / (norm_v1 * norm_v2)
        
        # Clip the value to avoid numerical errors that may arise due to floating point precision
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        
        # Calculate the angle in radians
        angle_radians = np.arccos(cos_angle)

        return angle_radians

    def getAbsoluteAngleDiff(self, angle1, angle2):
        diff = abs(angle1 - angle2)
        if diff > 180:
            diff = abs(diff - 360)

        return diff
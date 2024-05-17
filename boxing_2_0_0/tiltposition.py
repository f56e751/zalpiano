# this class represents current sandbag position
class TiltPosition():
    def __init__(self):
        self.xyAngle = 0    # rad
        self.tiltAngle = 0  # degree

    def setPosition(self, xyAngle, tiltAngle):
        self.xyAngle = xyAngle
        self.tiltAngle = tiltAngle

    def getXYAngle(self):
        return self.xyAngle
    
    def getTiltAngle(self):
        return self.tiltAngle
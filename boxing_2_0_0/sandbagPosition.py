class SandbagPosition():
    def __init__(self):
        self.x = 0
        self.y = 0
        # self.maxLength = maxLength

    def updatePosition(self, x, y):
        self.x = x
        self.y = y

    def getPosition(self):
        return (self.x, self.y)
class Point():
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y
    
    def updatePosition(self, x, y):
        self.x = x
        self.y = y

    def getPosition(self):
        return (self.x, self.y)
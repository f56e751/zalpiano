from point import Point



class Node():
    def __init__(self, point: Point):
        if not isinstance(point, Point):
            raise TypeError("point is not an instance of SandbagPosition")
        self.point = point
        self.adjacentNodes = []
        self.cost = 0

    def getPoint(self):
        return self.point

    def getAdjacentNode(self):
        return self.adjacentNodes
    
    def isCostHigherThanCriticalCost(self, criticalCost):
        return self.cost > criticalCost
    
    def isCostHigherThanOtherNode(self, node):
        return self.cost > node.getCost()
    
    def updateCost(self, cost):
        self.cost = cost
    
    def updateAdjacentNode(self, node):
        if not isinstance(node, Node):
            raise TypeError("The provided object is not an instance of Node.")
        
        if node not in self.adjacentNodes:
            self.adjacentNodes.append(node)
    
    # def findMinNodeHigherThanCriticalCost(self, criticalCost):
    #     # return Node cost has to be smaller than this node cost
    #     for adjacentNode in self.adjacentNodes:


    # def findMaxNodeLowerThanCriticalCost(self, criticalCost):
    #     # return Node cost has to be larger than this node cost

    def findMinNodeHigherThanCriticalCost(self, criticalCost):
        # Initialize with None and infinity to help find the minimum cost node above critical cost
        # return 값이 None이면 재귀함수 종료
        minNode = None
        minCost = float('inf')
        for adjacentNode in self.adjacentNodes:
            if adjacentNode.cost > criticalCost and adjacentNode.cost < minCost:
                minCost = adjacentNode.cost
                minNode = adjacentNode
        return minNode

    def findMaxNodeLowerThanCriticalCost(self, criticalCost):
        # Initialize with None and negative infinity to help find the maximum cost node below critical cost
        maxNode = None
        maxCost = float('-inf')
        for adjacentNode in self.adjacentNodes:
            if adjacentNode.cost < criticalCost and adjacentNode.cost > maxCost:
                maxCost = adjacentNode.cost
                maxNode = adjacentNode
        return maxNode

    def getCost(self):
        return self.cost

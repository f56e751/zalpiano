from punchcost import PunchCost
from sandbagPosition import SandbagPosition
import numpy as np

class OptimalAction():
    def __init__(self, PunchCost:PunchCost, SandbagPosition : SandbagPosition, criticalDistance, semicriticalDistance):
        self.Punchcost = PunchCost
        self.criticalDistance = criticalDistance
        self.semicriticalDistance = semicriticalDistance
        self.SandbagPosition = SandbagPosition
        self.count = 0
        self.criticalCost = 0.2
    # def getOptimalAction(self, leftCoordinate, rightCoordinate, centerCoordinate):
    #     leftDistance = np.sqrt((leftCoordinate[0] - centerCoordinate[0])**2 + (leftCoordinate[1] - centerCoordinate[1])**2)
    #     rightDistance = np.sqrt((rightCoordinate[0] - centerCoordinate[0])**2 + (rightCoordinate[1] - centerCoordinate[1])**2)
        
    #     closeDistance = leftDistance if leftDistance < rightDistance else rightDistance
    #     if closeDistance >= self.semicriticalDistance:
    #         return (0,0)
    #     elif closeDistance < self.semicriticalDistance and closeDistance > self.criticalDistance:
    #         meshSize = self.Punchcost.getMeshSize()
    #         interpolatedNum = meshSize * int((closeDistance - self.criticalDistance) / (self.semicriticalDistance - self.criticalDistance))
    #         return self.Punchcost.find_nth_lowest_cost_point(interpolatedNum)
    #     else:
    #         return self.Punchcost.find_lowest_cost_point()
    def getSemicriticalDistance(self):
        return self.semicriticalDistance

    def getOptimalAction(self, leftCoordinate, rightCoordinate, centerCoordinate):
        leftDistance = np.sqrt((leftCoordinate[0] - centerCoordinate[0])**2 + (leftCoordinate[1] - centerCoordinate[1])**2)
        rightDistance = np.sqrt((rightCoordinate[0] - centerCoordinate[0])**2 + (rightCoordinate[1] - centerCoordinate[1])**2)
        
        closeDistance = leftDistance if leftDistance < rightDistance else rightDistance
        if closeDistance >= self.semicriticalDistance:
            # print(f"position 0 {self.count} times")
            self.count += 1
            return (0,0)
        # elif closeDistance < self.semicriticalDistance and closeDistance > self.criticalDistance:
        #     meshSize = self.Punchcost.getMeshSize()
        #     interpolatedNum = meshSize * int((closeDistance - self.criticalDistance) / (self.semicriticalDistance - self.criticalDistance))
        #     return self.Punchcost.find_nth_lowest_cost_point(interpolatedNum)
        else:
            return self.Punchcost.find_lowest_cost_point()
        

    # def getOptimalAction_use_currentPosition(self,):
    #     currentPosition = self.SandbagPosition.getPosition()
    #     curretnCost = self.Punchcost.get_cost_at_point(currentPosition[0], currentPosition[1])
    #     if curretnCost > self.criticalCost:



    def getOptimalAction_use_currentPosition(self, leftCoordinate, rightCoordinate, centerCoordinate):


        leftDistance = np.sqrt((leftCoordinate[0] - centerCoordinate[0])**2 + (leftCoordinate[1] - centerCoordinate[1])**2)
        rightDistance = np.sqrt((rightCoordinate[0] - centerCoordinate[0])**2 + (rightCoordinate[1] - centerCoordinate[1])**2)
        
        closeDistance = leftDistance if leftDistance < rightDistance else rightDistance
        if closeDistance >= self.semicriticalDistance:
            # print(f"position 0 {self.count} times")
            self.count += 1
            return (0,0)
        
    

        currentPosition = self.SandbagPosition.getPosition()
        currentCost = self.Punchcost.get_cost_at_point(currentPosition[0], currentPosition[1])

        if currentCost == None:
            return (0,0)

        if currentCost > self.criticalCost:
            # Find the point with the lowest cost that is closer than the current position
            min_cost = float('inf')
            min_pos = None
            min_distance = float('inf')

            # Iterate over all mesh points to find the optimal one
            for idx in range(self.Punchcost.xMesh.size):
                x, y = self.Punchcost.xMesh[idx], self.Punchcost.yMesh[idx]
                cost = self.Punchcost.C_total[idx]
                distance = np.sqrt((x - currentPosition[0])**2 + (y - currentPosition[1])**2)

                # Check if this point has a lower cost and is closer than any previously considered point
                if cost < min_cost or (cost == min_cost and distance < min_distance):
                    min_cost = cost
                    min_pos = (x, y)
                    min_distance = distance

            if min_pos:
                return min_pos
            else:
                # No suitable position found, maybe stay in place or handle differently
                return currentPosition
        else:
            # If the current cost is not critical, no movement needed
            return currentPosition


    # def getOptimalAction_use_currentPosition(self, leftCoordinate, rightCoordinate, centerCoordinate):
        # # Calculate distances from the current center to the left and right coordinates
        # leftDistance = np.sqrt((leftCoordinate[0] - centerCoordinate[0])**2 + (leftCoordinate[1] - centerCoordinate[1])**2)
        # rightDistance = np.sqrt((rightCoordinate[0] - centerCoordinate[0])**2 + (rightCoordinate[1] - centerCoordinate[1])**2)
        
        # # Determine the closest distance
        # closeDistance = min(leftDistance, rightDistance)
        # if closeDistance >= self.semicriticalDistance:
        #     self.count += 1
        #     return (0, 0)  # Return the center position if the closest distance is beyond the semicritical distance

        # # Get the current position and its cost
        # currentPosition = self.SandbagPosition.getPosition()
        # currentCost = self.Punchcost.get_cost_at_point(currentPosition[0], currentPosition[1])
        # print(f"currentCost is {currentCost}")
        # # If no cost data is available, return center
        # if currentCost is None:
        #     return (0, 0)

        # # Proceed if the current cost exceeds the critical cost threshold
        # if currentCost > self.criticalCost:
        #     # Initialize variables for finding the optimal position
        #     min_cost = float('inf')
        #     min_pos = None
        #     min_distance = float('inf')

        #     # Iterate over all mesh points to find the optimal position
        #     for idx in range(self.Punchcost.xMesh.size):
        #         x, y = self.Punchcost.xMesh[idx], self.Punchcost.yMesh[idx]
        #         cost = self.Punchcost.C_total[idx]
        #         distance = np.sqrt((x - currentPosition[0])**2 + (y - currentPosition[1])**2)

        #         # Check if the point is a viable move: it should have a cost less than or equal to the current cost and be closer
        #         if cost < min_cost and cost >= self.criticalCost and (cost == min_cost and distance < min_distance):
        #             min_cost = cost
        #             min_pos = (x, y)
        #             min_distance = distance

        #     # Return the new position if found, otherwise stay in the current position
        #     return min_pos if min_pos else currentPosition
        # else:
        #     # If the current cost does not warrant a move, stay in the current position
        #     return currentPosition

import numpy as np
import matplotlib.pyplot as plt
from coordinateTransformer import CoordinateTransformer
from coordinateTransformer import CoordinateTransformer_nottilt_to_tilt
from punchcostfunction import CostFunction
from point import Point
from punchcostfunction import SegmentCostFunction, LineCostFunction, SegmentCostFunction_out0


class PunchCost:
    def __init__(self, sigma=100.0, grid_size=11):
        # self.initialize_positions(x_h, y_h, x_fl, y_fl, x_fr, y_fr)
        self.sigma = sigma
        self.grid_size = grid_size

        self.x_h, self.y_h = 0 , 0
        self.x_fl, self.y_fl = 1 ,1
        self.x_fr, self.y_fr = 2 , 2
        
        # Create a grid of points
        self.maxLength = 100
        self.scale = self.maxLength / self.grid_size
        self.x = np.linspace(-self.maxLength, self.maxLength, self.grid_size)
        self.y = np.linspace(-self.maxLength, self.maxLength, self.grid_size)
        self.xMesh, self.yMesh = np.meshgrid(self.x, self.y)

        
        circle_mask = (self.xMesh**2 + self.yMesh**2 <= self.maxLength**2 * 1.1) & (self.yMesh < 0)
        self.xMesh = self.xMesh[circle_mask]
        self.yMesh = self.yMesh[circle_mask]

        if not np.any((self.xMesh == 0) & (self.yMesh == 0)):
            self.xMesh = np.append(self.xMesh, 0)
            self.yMesh = np.append(self.yMesh, 0)

        
        self.CoordinateTransformer_nottilt_to_tilt = CoordinateTransformer_nottilt_to_tilt()
        self.CostFunction = None
        self.cost_map = {}
    
        distance_between_points = (2 * self.maxLength) / (self.grid_size - 1) if self.grid_size > 1 else self.maxLength
        self.adjacentDistance = distance_between_points * np.sqrt(2) + 0.01

        # self.points = []
        self.points = {}
        self.initializePoints()
        
    def getAdjacentDistance(self):
        return self.adjacentDistance

    # def initializePoints(self):
    #     for i in range(len(self.xMesh)):
    #         self.points.append(Point(self.xMesh[i], self.yMesh[i]))

    #     # Check if (0,0) is already in points
    #     isZeroInPoints = any(np.isclose(point.getPosition()[0], 0) and np.isclose(point.getPosition()[1], 0) for point in self.points)
        
    #     if not isZeroInPoints:
    #         self.points.append(Point(0,0))

    def initializePoints(self):
        for x, y in zip(self.xMesh.flat, self.yMesh.flat):  # Ensure flat iteration
            point = Point(x, y)
            self.points[(x, y)] = point  # Use coordinates as keys for easy access

    def getOriginalPoint(self):
        # Return the Point object for the origin (0,0)
        zero_key = (0.0, 0.0)  # The key for the origin point
        return self.points.get(zero_key)  # Use get to safely return None if not prese


    def getPoints(self):
        return self.points.values()
    
    def getPoint(self, pointKey):
        return self.points[pointKey]


    def getMaxLength(self):
        return self.maxLength

    def getMeshSize(self):
        print(f"punchcost class -> self.xMesh.size is: {self.xMesh.size}")
        return self.xMesh.size

    # def initialize_cost_map(self, C_total):
    #     # Fill the cost map with coordinates and costs
    #     for i in range(len(self.xMesh)):
    #         self.cost_map[(self.xMesh[i], self.yMesh[i])] = C_total[i]

    def initialize_cost_map(self, C_total):
        # Fill the cost map with coordinates and costs using points from self.points
        if self.CostFunction is not None:
            index = 0
            for (x, y), point in self.points.items():
                if index < len(C_total):  # Ensure we do not go out of bounds
                    self.cost_map[point] = C_total[index]
                    index += 1

            

    def initializeCostFunction(self, CostFunction: CostFunction):
        self.CostFunction = CostFunction
    
    
    def calculate_total_cost(self, humanCoordinate, centerCoordinate, leftCoordinate, rightCoordinate, xAxisAngleDiff):
        assert self.CostFunction is not None, "CostFunction has not been initialized."
        # 받을때 이 좌표계로 변환해서 받기
        # xAxisAngleDiff: degree
        humanCoordinate = self.CoordinateTransformer_nottilt_to_tilt.transform(humanCoordinate, centerCoordinate, xAxisAngleDiff, 1)
        leftCoordinate  = self.CoordinateTransformer_nottilt_to_tilt.transform(leftCoordinate, centerCoordinate, xAxisAngleDiff, 1)
        rightCoordinate = self.CoordinateTransformer_nottilt_to_tilt.transform(rightCoordinate, centerCoordinate, xAxisAngleDiff, 1)
        # print(f"puncostClass.calcualte_total_cost() -> humanCoordinate is: {humanCoordinate}")
        # print(f"puncostClass.calcualte_total_cost() -> leftCoordinate is: {leftCoordinate}")
        self.CostFunction.initializePosition(humanCoordinate, leftCoordinate, rightCoordinate)
        self.C_total = self.CostFunction.calculate(self.xMesh, self.yMesh)
        self.initialize_cost_map(self.C_total)
        return self.C_total
    

    def get_cost_at_point(self, point:Point):
        
        try:
            # First, try to retrieve the cost from the hash table
            return self.cost_map[point]
        except KeyError:
            print(f"punchcost.py -> {point.getPosition()} not in self.cost_map")
            # If the point is not in the hash table, calculate the cost for this point
            print("class PunchCost.get_cost_at_point() -> point not in points")
            x , y = point.getPosition()
            calculated_cost = self.CostFunction.calculate_point_cost(x, y)
            # Update the hash table with the newly calculated cost
            self.cost_map[point] = calculated_cost
            return calculated_cost

        
    # def get_cost_at_point(self, x, y):
    #     # TODO 외부에서 불러올때 self.xMesh, self.yMesh에 있는 점들로 불러오기
    #     # 여기 없으면 다시 계산해서 비효율적
    #     """
    #     Get the cost at a specific point (x, y) by checking the hash table first,
    #     and if not present, calculate it for that specific point.
    #     """
    #     point = (x, y)
    #     try:
    #         # First, try to retrieve the cost from the hash table
    #         return self.cost_map[point]
    #     except KeyError:
    #         print(f"punchcost.py -> x : {x}, y: {y}")
    #         # If the point is not in the hash table, calculate the cost for this point
    #         print("class PunchCost.get_cost_at_point() -> point not in points")
    #         calculated_cost = self.CostFunction.calculate_point_cost(x, y)
    #         # Update the hash table with the newly calculated cost
    #         self.cost_map[point] = calculated_cost
    #         return calculated_cost

    def plot_cost_function(self):
        plt.figure(figsize=(10, 8))
        plt.contourf(self.xMesh, self.yMesh, self.C_total, levels=50, cmap='viridis')
        plt.plot([self.x_h, self.x_fl], [self.y_h, self.y_fl], 'r-', linewidth=2)  # plot the vector for the left fist
        plt.plot([self.x_h, self.x_fr], [self.y_h, self.y_fr], 'b-', linewidth=2)  # plot the vector for the right fist
        plt.scatter([self.x_h, self.x_fl, self.x_fr], [self.y_h, self.y_fl, self.y_fr], color='red')  # plot the head and fists
        plt.colorbar(label='Cost')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Cost Function Around Punch Trajectories')
        plt.show()

    def plot_cost_function_3D(self):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')  # 3D axes 생성
        surf = ax.plot_surface(self.xMesh, self.yMesh, self.C_total, cmap='viridis', edgecolor='none')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Cost')
        plt.title('3D View of Cost Function Around Punch Trajectories')
        fig.colorbar(surf, shrink=0.5, aspect=5)  # 컬러바 추가
        plt.show()
        
    def find_lowest_cost_point(self):
        min_cost_index = np.unravel_index(np.argmin(self.C_total, axis=None), self.C_total.shape)
        return (self.xMesh[min_cost_index], self.yMesh[min_cost_index])
    
    def find_nth_lowest_cost_point(self, n):
        if n > self.C_total.size:
            raise ValueError("n is larger than the number of elements in the cost matrix")
        
        # C_total에서 n번째로 작은 값의 인덱스 찾기
        # np.partition을 사용하여 n번째 값과 그보다 작은 값을 분할
        # np.argpartition은 인덱스를 반환하므로, 이를 사용하여 위치를 찾을 수 있음
        flattened_index = np.argpartition(self.C_total.ravel(), n-1)[n-1]
        # unravel_index를 사용하여 1차원 인덱스를 2차원 인덱스로 변환
        index = np.unravel_index(flattened_index, self.C_total.shape)
        return (self.xMesh[index], self.yMesh[index])
    

    def print_cost_map_keys(self):
        print("Keys in cost_map and their types:")
        for key in self.cost_map.keys():
            print(f"{key}: {type(key)}")


if __name__ == "__main__":
    # 인스턴스 생성
    punch_cost = PunchCost(sigma=20, grid_size=50)  # sigma 값과 grid 크기를 설정
    # punch_cost.calculate_total_cost([50,50],[100,100],[50,0],[0,50],0)
    # # 비용 함수를 시각화
    # punch_cost.plot_cost_function_3D()
    # punch_cost.plot_cost_function()

    # # 비용이 가장 낮은 지점 찾기
    # lowest_cost_point = punch_cost.find_lowest_cost_point()
    # print("Point with the lowest cost:", lowest_cost_point)
    # print(punch_cost.C_total.shape)



    punch_cost = PunchCost()
    points = punch_cost.getPoints()
    count = 0
    for point in points:
        count += 1
        print(point.getPosition())

    print(count)
    CostFunction = SegmentCostFunction("exponential", 1.2)
    punch_cost.initializeCostFunction(CostFunction)
    punch_cost.calculate_total_cost([50, 50], [100, 100], [50, 0], [0, 50], 0)  # Calculate costs to populate the cost_map
    print(punch_cost.adjacentDistance)
    punch_cost.print_cost_map_keys()
    point = (-20.0, -100.0)
    punch_cost.get_cost_at_point(point[0], point[1])
    # TODO (0,0)도 costmap에 넣기
    originalPoint = punch_cost.getOriginalPoint()
    print(originalPoint)
    print(originalPoint.getPosition())
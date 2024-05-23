import numpy as np
import matplotlib.pyplot as plt
from coordinateTransformer import CoordinateTransformer
from coordinateTransformer import CoordinateTransformer_nottilt_to_tilt
from punchcostfunction import CostFunction
from point import Point

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
        for x, y in zip(self.xMesh, self.yMesh):
            point_key = (round(x, 2), round(y, 2))  # round to ensure consistent hashing
            if point_key not in self.points:
                self.points[point_key] = Point(x, y)

        # Ensure (0,0) is always included in the points if not already present
        zero_key = (0.0, 0.0)
        if zero_key not in self.points:
            self.points[zero_key] = Point(0, 0)

    # def getPoints(self):
    #     return self.points

    def getPoints(self):
        return self.points.values()

    def getMaxLength(self):
        return self.maxLength

    def getMeshSize(self):
        print(f"punchcost class -> self.xMesh.size is: {self.xMesh.size}")
        return self.xMesh.size

    def initialize_cost_map(self, C_total):
        # Fill the cost map with coordinates and costs
        for i in range(len(self.xMesh)):
            self.cost_map[(self.xMesh[i], self.yMesh[i])] = C_total[i]

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
    
    # def get_cost_at_point(self, x, y):
    #     """
    #     Get the cost at a specific point (x, y).
    #     Args:
    #     x (float): The x-coordinate of the point.
    #     y (float): The y-coordinate of the point.

    #     Returns:
    #     float: The cost at the point if the point is within the mesh grid; otherwise, returns None.
    #     """
    #     # Check if the point is within the xMesh and yMesh
    #     try:
    #         # Find the index of the closest values in xMesh and yMesh to the given x, y
    #         x_idx = (np.abs(self.xMesh - x)).argmin()
    #         y_idx = (np.abs(self.yMesh - y)).argmin()

    #         # Check if the coordinates match exactly (or very close, considering numerical precision)
    #         if np.isclose(self.xMesh[x_idx], x) and np.isclose(self.yMesh[y_idx], y):
    #             # Retrieve and return the cost value at the found index
    #             return self.C_total[x_idx, y_idx]
    #         else:
    #             return None
    #     except ValueError:
    #         # If x or y is out of the bounds of xMesh or yMesh
    #         return None

    # def get_cost_at_point(self, x, y):
    #     # TODO 이 방식 말고 Hash table 이용해서 찾기
    #     """
    #     Get the cost at a specific point (x, y).
    #     Args:
    #     x (float): The x-coordinate of the point.
    #     y (float): The y-coordinate of the point.

    #     Returns:
    #     float: The cost at the point if the point is within the mesh grid; otherwise, returns None.
    #     """
    #     # Compute the distance from each point in the mesh to the specified point (x, y)
    #     distances = np.sqrt((self.xMesh - x)**2 + (self.yMesh - y)**2)
    #     # Find the index of the minimum distance
    #     min_idx = distances.argmin()

    #     # Check if the closest point's coordinates are close enough to (x, y)
    #     if np.isclose(self.xMesh[min_idx], x) and np.isclose(self.yMesh[min_idx], y):
    #         # Retrieve and return the cost value at the found index
    #         return self.C_total[min_idx]
    #     else:
    #         return None
        
    def get_cost_at_point(self, x, y):
        # TODO 외부에서 불러올때 self.xMesh, self.yMesh에 있는 점들로 불러오기
        # 여기 없으면 다시 계산해서 비효율적
        """
        Get the cost at a specific point (x, y) by checking the hash table first,
        and if not present, calculate it for that specific point.
        """
        point = (x, y)
        try:
            # First, try to retrieve the cost from the hash table
            return self.cost_map[point]
        except KeyError:
            # If the point is not in the hash table, calculate the cost for this point
            print("class PunchCost.get_cost_at_point() -> point not in points")
            calculated_cost = self.CostFunction.calculate_point_cost(x, y)
            # Update the hash table with the newly calculated cost
            self.cost_map[point] = calculated_cost
            return calculated_cost




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

    print(punch_cost.adjacentDistance)

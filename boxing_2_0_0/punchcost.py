import numpy as np
import matplotlib.pyplot as plt
from coordinateTransformer import CoordinateTransformer
from coordinateTransformer import CoordinateTransformer_nottilt_to_tilt

class PunchCost:
    def __init__(self, sigma=100.0, grid_size=10):
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
        
        self.CoordinateTransformer = CoordinateTransformer()
        self.CoordinateTransformer_nottilt_to_tilt = CoordinateTransformer_nottilt_to_tilt()

    
    def perpendicular_distance(self, x1, y1, x2, y2, X, Y):
        numerator = np.abs((y2 - y1) * X - (x2 - x1) * Y + x2 * y1 - y2 * x1)
        denominator = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        return numerator / denominator
    
    def calculate_total_cost(self, humanCoordinate, centerCoordinate, leftCoordinate, rightCoordinate, xAxisAngleDiff):
        # 받을때 이 좌표계로 변환해서 받기
        # xAxisAngleDiff: degree
        humanCoordinate = self.CoordinateTransformer_nottilt_to_tilt.transform(humanCoordinate, centerCoordinate, xAxisAngleDiff, 1)
        leftCoordinate  = self.CoordinateTransformer_nottilt_to_tilt.transform(leftCoordinate, centerCoordinate, xAxisAngleDiff, 1)
        rightCoordinate = self.CoordinateTransformer_nottilt_to_tilt.transform(rightCoordinate, centerCoordinate, xAxisAngleDiff, 1)
        # print(f"puncostClass.calcualte_total_cost() -> humanCoordinate is: {humanCoordinate}")
        # print(f"puncostClass.calcualte_total_cost() -> leftCoordinate is: {leftCoordinate}")
        
        self.x_h, self.y_h = humanCoordinate[0], humanCoordinate[1]
        self.x_fl, self.y_fl = leftCoordinate[0], leftCoordinate[1]
        self.x_fr, self.y_fr = rightCoordinate[0], rightCoordinate[1]
        
        distance_left = self.perpendicular_distance(self.x_h, self.y_h, self.x_fl, self.y_fl, self.xMesh, self.yMesh)
        distance_right = self.perpendicular_distance(self.x_h, self.y_h, self.x_fr, self.y_fr, self.xMesh, self.yMesh)
        
        C_left = np.exp(-distance_left**2 / (2 * self.sigma**2))
        C_right = np.exp(-distance_right**2 / (2 * self.sigma**2))
        self.C_total = C_left + C_right
        return C_left + C_right
    
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


if __name__ == "__main__":
    # Create an instance of the class and use its methods
    # punch_cost = PunchCost()
    # punch_cost.plot_cost_function()
    # lowest_cost_point = punch_cost.find_lowest_cost_point()
    # print("Point with the lowest cost:", lowest_cost_point)


    # 먼저 PunchCost 클래스를 정의해야 합니다. 위에서 이미 제공된 클래스를 사용합니다.
    # 이제 테스트 코드를 작성합니다.

    # 인스턴스 생성
    punch_cost = PunchCost(sigma=20, grid_size=50)  # sigma 값과 grid 크기를 설정
    punch_cost.calculate_total_cost([50,50],[100,100],[50,0],[0,50],0)
    # 비용 함수를 시각화
    punch_cost.plot_cost_function_3D()
    punch_cost.plot_cost_function()

    # 비용이 가장 낮은 지점 찾기
    lowest_cost_point = punch_cost.find_lowest_cost_point()
    print("Point with the lowest cost:", lowest_cost_point)
    print(punch_cost.C_total.shape)

    # 이 코드는 matplotlib을 사용하여 비용 분포를 시각화합니다.
    # 'viridis' 컬러맵을 사용하여 더 높은 비용은 더 밝은 색상으로, 낮은 비용은 어두운 색상으로 표시됩니다.
    # 또한, 최소 비용 지점도 출력됩니다.

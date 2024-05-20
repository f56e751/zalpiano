import numpy as np
import matplotlib.pyplot as plt

class PunchCost:
    def __init__(self, sigma=1.0, grid_size=10):
        # self.initialize_positions(x_h, y_h, x_fl, y_fl, x_fr, y_fr)
        self.sigma = sigma
        self.grid_size = grid_size

        self.x_h, self.y_h = 0 , 0
        self.x_fl, self.y_fl = 1 ,1
        self.x_fr, self.y_fr = 2 , 2
        
        # Create a grid of points
        self.maxLength = 100
        self.x = np.linspace(-self.maxLength, self.maxLength, self.grid_size)
        self.y = np.linspace(-self.maxLength, self.maxLength, self.grid_size)
        self.xMesh, self.yMesh = np.meshgrid(self.x, self.y)
        
        # Calculate the total cost
        self.C_total = self.calculate_total_cost()
    
    def initialize_positions(self, humanCoordinate, leftCoordinate, rightCoordinate):
        self.x_h, self.y_h = humanCoordinate[0], humanCoordinate[1]
        self.x_fl, self.y_fl = leftCoordinate[0], leftCoordinate[1]
        self.x_fr, self.y_fr = rightCoordinate[0], rightCoordinate[1]
    
    def perpendicular_distance(self, x1, y1, x2, y2, X, Y):
        numerator = np.abs((y2 - y1) * X - (x2 - x1) * Y + x2 * y1 - y2 * x1)
        denominator = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        return numerator / denominator
    
    def calculate_total_cost(self):
        distance_left = self.perpendicular_distance(self.x_h, self.y_h, self.x_fl, self.y_fl, self.xMesh, self.yMesh)
        distance_right = self.perpendicular_distance(self.x_h, self.y_h, self.x_fr, self.y_fr, self.xMesh, self.yMesh)
        
        C_left = np.exp(-distance_left**2 / (2 * self.sigma**2))
        C_right = np.exp(-distance_right**2 / (2 * self.sigma**2))
        
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
        
    def find_lowest_cost_point(self):
        min_cost_index = np.unravel_index(np.argmin(self.C_total, axis=None), self.C_total.shape)
        return (self.xMesh[min_cost_index], self.yMesh[min_cost_index])

# Create an instance of the class and use its methods
# punch_cost = PunchCost()
# punch_cost.plot_cost_function()
# lowest_cost_point = punch_cost.find_lowest_cost_point()
# print("Point with the lowest cost:", lowest_cost_point)

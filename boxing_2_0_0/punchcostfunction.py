import numpy as np

class CostFunction:
    def __init__(self, function_type):
        self.function_types = ["exponential", "inverse"]
        self.function_type = function_type
        self.sigma = 1
    
        if self.function_type not in self.function_types:
            raise ValueError(f"Invalid function_type provided. Choose from {self.function_types}")

    def setSigma(self, sigma):
        self.sigma = sigma
    
    def initializePosition(self,humanCoordinate, leftCoordinate, rightCoordinate):
        self.humanCoordinate = humanCoordinate
        self.leftCoordinate  = leftCoordinate
        self.rightCoordinate = rightCoordinate

    def calculate(self, xMesh, yMesh):
        raise NotImplementedError("Each cost function must implement the 'calculate' method.")
    
    def exponential(self, value):
        return np.exp(-value**2 / (2 * self.sigma**2))
    
    def inverse(self, value):
        return 1 / value
    
    def getCost(self, value):
        if self.function_type == "exponential":
            return self.exponential(value)
        elif self.function_type == "inverse":
            return self.inverse(value)
        

    def calculate_point_cost(self, x, y):
        # This method should calculate cost for a single point.
        # Implement the specific logic based on the cost function type.
        if self.function_type == "exponential":
            distance = np.sqrt((x - self.humanCoordinate[0])**2 + (y - self.humanCoordinate[1])**2)
            return self.exponential(distance)
        elif self.function_type == "inverse":
            distance = np.sqrt((x - self.humanCoordinate[0])**2 + (y - self.humanCoordinate[1])**2)
            return self.inverse(distance)

class SegmentCostFunction(CostFunction):
    def __init__(self, function_type, scale):
        super().__init__(function_type)  # 기반 클래스 생성자 호출
        self.scale = scale  # 추가 속성 초기화

    def calculate(self, xMesh, yMesh):
        C_left = self.point_to_segment_distance(self.humanCoordinate, self.leftCoordinate ,xMesh, yMesh)
        C_right = self.point_to_segment_distance(self.humanCoordinate, self.rightCoordinate ,xMesh, yMesh)
        C_total = self.getCost(C_left) + self.getCost(C_right)
        return C_total
    
    def point_to_segment_distance(self, humanCoordinate, handCoordinate, X, Y):
        x1, y1 = humanCoordinate[0], humanCoordinate[1]
        x2, y2 = handCoordinate[0], handCoordinate[1]
        
        # 원래의 선분 길이를 기반으로 scale배 늘린 끝점 계산
        dx = x2 - x1
        dy = y2 - y1
        length = np.sqrt(dx**2 + dy**2) * self.scale  # 원래 길이에 scale을 곱함
        x2_scaled = x1 + (dx / np.sqrt(dx**2 + dy**2)) * length
        y2_scaled = y1 + (dy / np.sqrt(dx**2 + dy**2)) * length
        
        dx_scaled = x2_scaled - x1
        dy_scaled = y2_scaled - y1
        d_squared = dx_scaled**2 + dy_scaled**2
        p_dx = X - x1
        p_dy = Y - y1

        if d_squared == 0:
            return np.sqrt(p_dx**2 + p_dy**2)

        t = (p_dx * dx_scaled + p_dy * dy_scaled) / d_squared
        closest_x = np.where(t < 0, x1, np.where(t > 1, x2_scaled, x1 + t * dx_scaled))
        closest_y = np.where(t < 0, y1, np.where(t > 1, y2_scaled, y1 + t * dy_scaled))

        distance = np.sqrt((X - closest_x)**2 + (Y - closest_y)**2)
        return distance
    
# class SegmentCostFunction_out0(CostFunction):
#     def __init__(self, function_type, scale):
#         super().__init__(function_type)  # 기반 클래스 생성자 호출
#         self.scale = scale  # 추가 속성 초기화

#     def calculate(self, xMesh, yMesh):
#         C_left = self.point_to_segment_distance(self.humanCoordinate, self.leftCoordinate, xMesh, yMesh)
#         C_right = self.point_to_segment_distance(self.humanCoordinate, self.rightCoordinate, xMesh, yMesh)
#         # 코스트 함수로 각 점에 대한 코스트 계산
#         C_total = self.getCost(C_left) + self.getCost(C_right)
#         return C_total
    
#     def point_to_segment_distance(self, humanCoordinate, handCoordinate, X, Y):
#         x1, y1 = humanCoordinate[0], humanCoordinate[1]
#         x2, y2 = handCoordinate[0], handCoordinate[1]
        
#         # 선분의 길이를 scale배 늘림
#         dx = x2 - x1
#         dy = y2 - y1
#         length = np.sqrt(dx**2 + dy**2) * self.scale  # 원래 길이에 scale을 곱함
#         x2_scaled = x1 + (dx / np.sqrt(dx**2 + dy**2)) * length
#         y2_scaled = y1 + (dy / np.sqrt(dx**2 + dy**2)) * length
        
#         dx_scaled = x2_scaled - x1
#         dy_scaled = y2_scaled - y1
#         d_squared = dx_scaled**2 + dy_scaled**2
#         p_dx = X - x1
#         p_dy = Y - y1

#         if d_squared == 0:
#             return np.sqrt(p_dx**2 + p_dy**2)

#         t = (p_dx * dx_scaled + p_dy * dy_scaled) / d_squared
#         if t < 0 or t > 1:
#             # 선분의 양 끝점을 넘어서는 경우, 거리를 0으로 설정
#             return 0
#         else:
#             closest_x = x1 + t * dx_scaled
#             closest_y = y1 + t * dy_scaled
#             distance = np.sqrt((X - closest_x)**2 + (Y - closest_y)**2)
#             return distance


class SegmentCostFunction_out0(CostFunction):
    def __init__(self, function_type, scale):
        super().__init__(function_type)  # 기반 클래스 생성자 호출
        self.scale = scale  # 추가 속성 초기화

    def calculate(self, xMesh, yMesh):
        C_left = self.point_to_segment_distance(self.humanCoordinate, self.leftCoordinate, xMesh, yMesh)
        C_right = self.point_to_segment_distance(self.humanCoordinate, self.rightCoordinate, xMesh, yMesh)
        C_total = self.getCost(C_left) + self.getCost(C_right)
        return C_total
    
    def point_to_segment_distance(self, humanCoordinate, handCoordinate, X, Y):
        x1, y1 = humanCoordinate[0], humanCoordinate[1]
        x2, y2 = handCoordinate[0], handCoordinate[1]
        
        # 선분의 길이를 scale배 늘림
        dx = x2 - x1
        dy = y2 - y1
        length = np.sqrt(dx**2 + dy**2) * self.scale  # 원래 길이에 scale을 곱함
        x2_scaled = x1 + (dx / np.sqrt(dx**2 + dy**2)) * length
        y2_scaled = y1 + (dy / np.sqrt(dx**2 + dy**2)) * length
        
        dx_scaled = x2_scaled - x1
        dy_scaled = y2_scaled - y1
        d_squared = dx_scaled**2 + dy_scaled**2
        p_dx = X - x1
        p_dy = Y - y1

        if d_squared == 0:
            return np.sqrt(p_dx**2 + p_dy**2)

        t = (p_dx * dx_scaled + p_dy * dy_scaled) / d_squared
        distance = np.where((t < 0) | (t > 1), 0,
                            np.sqrt((X - (x1 + t * dx_scaled))**2 + (Y - (y1 + t * dy_scaled))**2))
        return distance


        
class LineCostFunction(CostFunction):
    def calculate(self, xMesh, yMesh):
        C_left = self.perpendicular_distance(self.humanCoordinate, self.leftCoordinate, xMesh, yMesh)
        C_right = self.perpendicular_distance(self.humanCoordinate, self.rightCoordinate, xMesh, yMesh)
        C_total = self.getCost(C_left) + self.getCost(C_right)
        return C_total

    def perpendicular_distance(self, humanCoordinate, handCoordinate, X, Y):
        x1, y1 = humanCoordinate[0], humanCoordinate[1]
        x2, y2 = handCoordinate[0], handCoordinate[1]
        numerator = np.abs((y2 - y1) * X - (x2 - x1) * Y + x2 * y1 - y2 * x1)
        denominator = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        return numerator / denominator
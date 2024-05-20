import numpy as np
import matplotlib.pyplot as plt

class CoordinateTransformer:
    def initialize(self, origin, angle):
        self.origin = np.array(origin)
        self.angle = angle
        # 회전 행렬 수정: y축 방향 반대
        self.rotation_matrix = np.array([
            [np.cos(self.angle), -np.sin(self.angle)],
            [np.sin(self.angle), np.cos(self.angle)]
        ])
        self.rotatedPoints = []

    def transform(self, points):
        points = np.array(points)
        # 회전 적용 후 y 좌표 반전
        rotated_points = np.dot(points, self.rotation_matrix.T)
        # 평행 이동 적용
        transformed_points = rotated_points + self.origin
        return transformed_points
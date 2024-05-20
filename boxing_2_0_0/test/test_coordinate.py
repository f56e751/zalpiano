import numpy as np
import matplotlib.pyplot as plt

class CoordinateTransformer:
    def __init__(self, origin, angle):
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
        self.rotatedPoints = rotated_points
        # 평행 이동 적용
        transformed_points = rotated_points + self.origin
        return transformed_points

# 원점 (1, -2)로 이동하고 45도 회전하는 변환기 생성 (y 축 부호 반대)
transformer = CoordinateTransformer((1, -2), np.radians(45))

# 변환할 좌표 설정
original_points = np.array([
    [0, 0],
    [1, 0],
    [1, 1],
    [0, 1]
])

# 좌표 변환 수행
transformed_points = transformer.transform(original_points)

# # 변환 전후 좌표를 플로팅
# plt.figure(figsize=(8, 8))
# plt.plot(original_points[:, 0], original_points[:, 1], 'ro-', label='Original Coordinates')
# plt.plot(transformed_points[:, 0], transformed_points[:, 1], 'bo-', label='Transformed Coordinates')

# # 각 점에 대해 레이블 추가
# for (x, y), (tx, ty) in zip(original_points, transformed_points):
#     plt.text(x, -y, f'({x},{y})', fontsize=9, ha='right')  # Y축 반전 표시
#     plt.text(tx, -ty, f'({tx:.2f},{ty:.2f})', fontsize=9, ha='right')  # Y축 반전 표시

# # 그래프 세팅
# plt.axhline(0, color='grey', lw=0.5)
# plt.axvline(0, color='grey', lw=0.5)
# plt.legend()
# plt.grid(True)
# plt.axis('equal')
# plt.title('Coordinate Transformation Visualization (OpenCV Style)')
# plt.xlabel('X axis')
# plt.ylabel('Y axis')
# plt.gca().invert_yaxis()  # Y축 뒤집기
# plt.show()

import cv2
import numpy as np
# 이미지 생성
height, width = 1000, 1000
image = np.zeros((height, width, 3), dtype=np.uint8)

# 원점 (150, 150)으로 이동하고 45도 회전하는 변환기 생성
transformer = CoordinateTransformer((150, 150), np.radians(45))

# 변환할 좌표 설정 (원점을 이미지의 중앙으로 잡음)
original_points = np.array([
    [100, 100],
    [200, 100],
    [200, 200],
    [100, 200]
])
num_points = 4
# 좌표 변환 수행
transformed_points = transformer.transform(original_points).astype(int)
# # 변환 전 좌표를 이미지에 빨간색으로 그리기
# for point in original_points:
#     cv2.circle(image, (point[0], point[1]), 5, (0, 0, 255), -1)  # Red color

# # 변환 후 좌표를 이미지에 파란색으로 그리기
# for point in transformed_points:
#     cv2.circle(image, tuple(point), 5, (255, 0, 0), -1)  # Blue color



rotated_points = transformer.rotatedPoints 

for i, point in enumerate(original_points):
    intensity = 255 - i * 255 // (num_points - 1)  # 점점 옅어지게
    cv2.circle(image, (point[0], point[1]), 5, (intensity, 0, 0), -1)  # Red gradient
    cv2.putText(image, f"{point[0]}, {point[1]}", (point[0] + 10, point[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (intensity, 255, 255), 1)


# 변환 후 좌표를 이미지에 그라데이션 색으로 그리기
for i, point in enumerate(transformed_points):
    intensity = 255 - i * 255 // (num_points - 1)  # 점점 옅어지게
    cv2.circle(image, tuple(point), 5, (0, 0, intensity), -1)  # Blue gradient
    cv2.putText(image, f"{point[0]}, {point[1]}", (point[0] + 10, point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, intensity), 1)

# 회전된 좌표를 이미지에 녹색으로 그리기
for i, point in enumerate(rotated_points.astype(int)):
    intensity = 255 - i * 255 // (num_points - 1)  # 점점 옅어지게
    cv2.circle(image, tuple(point), 5, (0, intensity, 0), -1)  # Green gradient
    # cv2.putText(image, f"{point[0]}, {point[1]}", (point[0] + 160, point[1] + 140),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, intensity, 255), 1)


# 이미지 표시
cv2.imshow('Transformed Coordinates', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

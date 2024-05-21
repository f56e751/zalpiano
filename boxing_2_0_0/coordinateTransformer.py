import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

class CoordinateTransformer:
    def transform(self, origin, angle, points):
        self.origin = np.array(origin)
        self.angle = angle
        # 회전 행렬 수정: y축 방향 반대
        self.rotation_matrix = np.array([
            [np.cos(self.angle), -np.sin(self.angle)],
            [np.sin(self.angle), np.cos(self.angle)]
        ])
        self.rotatedPoints = []
        points = np.array(points)
        # 회전 적용 후 y 좌표 반전
        rotated_points = np.dot(points, self.rotation_matrix.T)
        # 평행 이동 적용
        transformed_points = rotated_points + self.origin
        return transformed_points



class CoordinateTransformer_nottilt_to_tilt:
    def __init__(self):
        """
        Initialize the coordinate transformer without any specific origin, angle, or scale.
        These parameters will be set when a point transformation is requested.
        """
        pass  # No initial properties to set

    def calculate_angle_with_x_axis(self, v):
        # 라디안 단위로 각도 계산
        radians = np.arctan2(v[1], v[0])
        # 라디안을 도(degree) 단위로 변환
        degrees = np.degrees(radians)
        return degrees

    def transform(self, point, origin, angle, scale):
        """
        Transform a point from the xy plane to the x'y' plane using the specified origin, angle, and scale.

        Parameters:
        point : tuple
            The (x, y) coordinates of the point in the original xy plane.
        origin : tuple
            The (x, y) coordinates of the new origin in the xy plane.
        angle : float
            The angle in degrees by which the x axis is rotated to become the x' axis.
        scale : float
            The scaling factor R, which scales distances in the x'y' plane relative to the xy plane.

        Returns:
        tuple
            The transformed (x', y') coordinates in the new x'y' plane.
        """
        origin = np.array(origin)
        angle_radians = np.radians(angle)  # Convert angle from degrees to radians
        translated_point = np.array(point) - origin
        point_axis_radians = np.radians(self.calculate_angle_with_x_axis(translated_point))
        length = np.sqrt(translated_point[0] ** 2 + translated_point[1] ** 2)

        diff = point_axis_radians - angle_radians
        transformed_point = np.array([length * np.cos(diff), length * np.sin(diff)])
        # print(transformed_point)
        scaled_point = scale * transformed_point
        
        return tuple(scaled_point)


if __name__ == "__main__":
    transformer = CoordinateTransformer_nottilt_to_tilt()

    # Define the original point and transformation parameters
    original_point = (2, 3)
    origin = (1, 1)
    angle = 45  # degrees
    scale = 2

    # Transform the point
    transformed_point = transformer.transform(original_point, origin, angle, scale)
    print(transformed_point)
    # Plotting
    fig, ax = plt.subplots()
    # Original point and axes
    ax.plot(*original_point, 'ro', label='Original Point')
    ax.add_patch(FancyArrowPatch((0, 0), (5, 0), color='blue', arrowstyle='->', label='Original X-axis'))
    ax.add_patch(FancyArrowPatch((0, 0), (0, 5), color='green', arrowstyle='->', label='Original Y-axis'))

    # Transformed point and axes
    ax.plot(*transformed_point, 'go', label='Transformed Point')
    angle_rad = np.radians(angle)
    end_x = np.array([5 * np.cos(angle_rad), 5 * np.sin(angle_rad)]) + origin
    end_y = np.array([-5 * np.sin(angle_rad), 5 * np.cos(angle_rad)]) + origin
    ax.add_patch(FancyArrowPatch(origin, end_x, color='orange', arrowstyle='->', label='Transformed X-axis'))
    ax.add_patch(FancyArrowPatch(origin, end_y, color='purple', arrowstyle='->', label='Transformed Y-axis'))

    # Setting plot limits and labels
    ax.set_xlim(-1, 10)
    ax.set_ylim(-1, 10)
    ax.set_aspect('equal')
    ax.legend()
    plt.grid(True)
    plt.show()


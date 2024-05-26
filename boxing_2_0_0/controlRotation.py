import math
import numpy as np
class ControlRotation():
    def __init__(self, maximumPhi, maximumLength):
        # maximumPhi: rad
        # maximumLength: use this value to transform Length to xyPlaneRadius = 1
        #                abs(x), abs(y) <= maximumLength
        self.axis0_sign = 1
        self.axis1_sign = -1
        self.gearRatio = 10
        self.xyPlaneRadius = 1
        self.maximumLength = maximumLength
        self.fixed_radius = self.xyPlaneRadius / np.sin(maximumPhi) #length of rod

    def setMaxLength(self, maxLength):
        self.maximumLength = maxLength

    def calculate_spherical_coordinates(self, x, y):
        # transform x, y to unit Length
        distance_from_origin = math.sqrt(x**2 + y**2)
        if distance_from_origin * 0.8 > self.maximumLength:
            print(f"distance_from_origin is {distance_from_origin}")
            print(f"self.maximumLength is {self.maximumLength}")
            raise ValueError("The distance from the origin to (x, y) must be smaller than or equal to self.maximumLength")
        
        tf_x = x / self.maximumLength
        tf_y = y / self.maximumLength
        r_xy = math.sqrt(tf_x**2 + tf_y**2)
        theta = math.atan2(y, x)
        phi = math.asin(r_xy / self.fixed_radius)  # r_xy should be less than or equal to fixed_radius
        return theta, phi

    def rotateAnyPoint(self, x, y, relative_heading):
        theta, phi = self.calculate_spherical_coordinates(x, y)
        tan = math.tan(phi)
        sin = math.sin(theta + relative_heading - np.pi / 2)
        cos = math.cos(theta + relative_heading - np.pi / 2)
        axis0_degree = self.axis0_sign * math.degrees(math.atan(tan * cos))
        axis1_degree = self.axis1_sign * math.degrees(math.atan(tan * sin))
        return axis0_degree, axis1_degree

    def degreeToPose(self, degree):
        return degree / 360 * self.gearRatio
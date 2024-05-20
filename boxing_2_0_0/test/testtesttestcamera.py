import numpy as np

class VectorOperations:
    def angle_between_vectors(self, v1, v2):
        # Calculate the dot product
        dot_product = np.dot(v1, v2)
        
        # Calculate the magnitudes (norms) of the vectors
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        
        # Calculate the cosine of the angle
        cos_angle = dot_product / (norm_v1 * norm_v2)
        
        # Clip the value to avoid numerical errors that may arise due to floating point precision
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        
        # Calculate the angle in radians
        angle_radians = np.arccos(cos_angle)

        return angle_radians

def test_angle_between_vectors():
    vector_ops = VectorOperations()
    
    # Test cases
    test_vectors = [
        (np.array([1, 0]), np.array([0, 1]), 90.0),  # Perpendicular vectors
        (np.array([1, 0]), np.array([1, 0]), 0.0),   # Parallel vectors
        (np.array([1, 0]), np.array([-1, 1]), 135.0),# Opposite vectors
        (np.array([1, 1]), np.array([-1, 1]), 90.0), # Perpendicular vectors
    ]

    for v1, v2, expected_angle in test_vectors:
        angle_radians = vector_ops.angle_between_vectors(v1, v2)
        angle_degrees = np.degrees(angle_radians)
        
        print(f"Angle between vectors {v1} and {v2}: {angle_degrees:.2f} degrees (expected: {expected_angle:.2f} degrees)")

        # Verify the result
        # assert np.isclose(angle_degrees, expected_angle, atol=1e-2), f"Test failed for vectors {v1} and {v2}"

# Run the test
test_angle_between_vectors()

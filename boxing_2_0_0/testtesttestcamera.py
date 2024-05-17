import numpy as np
import time

# Function to test the speed of np.roots
def test_np_roots_speed(num_polynomials, polynomial_degree):
    # Generate random polynomial coefficients
    coefficients = np.random.rand(num_polynomials, polynomial_degree + 1)

    # Start the timer
    start_time = time.time()

    # Find roots of each polynomial
    roots = np.roots([1,4,4])

    # Stop the timer
    end_time = time.time()

    # Calculate the elapsed time
    elapsed_time = end_time - start_time

    print(f"Time taken to find roots of {num_polynomials} polynomials of degree {polynomial_degree}: {elapsed_time:.4f} seconds")
    print(f'root is {roots}')
    print(np.iscomplex(roots[0]))
# Example usage
num_polynomials = 1000  # Number of polynomials to test
polynomial_degree = 10  # Degree of each polynomial

test_np_roots_speed(num_polynomials, polynomial_degree)

a = np.array([1,3])
b = np.array([2,5])

print(np.dot(a,b))

rotation = np.array(([0,1],[-1,0]))
print(rotation)
print(np.dot([1,-1],rotation))


import numpy as np

def angle_between_vectors(v1, v2):
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
    
    # Convert the angle to degrees
    angle_degrees = np.degrees(angle_radians)
    
    return angle_radians, angle_degrees

# Example usage
v1 = np.array([1, 0])
v2 = np.array([-1, -1])

angle_radians, angle_degrees = angle_between_vectors(v1, v2)
print(f"Angle between vectors in radians: {angle_radians}")
print(f"Angle between vectors in degrees: {angle_degrees}")

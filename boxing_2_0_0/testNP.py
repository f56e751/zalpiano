import numpy as np
import matplotlib.pyplot as plt

def plot_arctan2():
    # Create a grid of x and y values
    x = np.linspace(-10, 10, 400)
    y = np.linspace(-10, 10, 400)
    
    # Create a meshgrid which will generate a grid of coordinates where the rows are copies of x
    # and columns are copies of y
    X, Y = np.meshgrid(x, y)
    
    # Calculate arctan2 for each combination of x and y values
    angles = np.arctan2(Y, X)
    
    # Convert angles from radians to degrees
    angles_deg = np.degrees(angles)
    
    # Plotting
    plt.figure(figsize=(8, 8))
    contour = plt.contourf(X, Y, angles_deg, levels=np.linspace(-180, 180, 37), cmap='hsv')
    plt.colorbar(contour, label='Angle (degrees)')
    plt.title('Arctan2(y, x) Results')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.grid(True)
    plt.axis('square')
    
    # Adding a color bar to show the angle values
    plt.colorbar(contour, label='Angle (degrees)')
    plt.show()

plot_arctan2()

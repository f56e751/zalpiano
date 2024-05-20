import numpy as np
grid_size = 100
x = np.linspace(-5, 5, grid_size)
y = np.linspace(-5, 5, grid_size)
xMesh, yMesh = np.meshgrid(x, y)

print(xMesh)
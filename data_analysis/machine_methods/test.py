import numpy as np
from scipy.interpolate import griddata, interp2d
import matplotlib.pyplot as plt

interp2d()
# Sample x, y, z data
x = np.array([0, 1, 2, 3, 4])
y = np.array([0, 1, 2, 3, 4])
z = np.array([1, 2, 3, 4, 5])

# Length to which we want to interpolate
length = 10

# Generate a grid of x, y values
xi = np.linspace(min(x), max(x), length)
yi = np.linspace(min(y), max(y), length)
xi, yi = np.meshgrid(xi, yi)

# Interpolate the z values over the grid
zi = griddata((x, y), z, (xi, yi), method='linear')

# Plotting the result
plt.imshow(zi, extent=(min(x), max(x), min(y), max(y)), origin='lower', cmap='viridis')
plt.colorbar()
plt.scatter(x, y, c=z, edgecolors='w')
plt.title('Interpolated 2D Array')
plt.show()

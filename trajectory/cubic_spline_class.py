import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

class CubicSplinePath:
    def __init__(self, waypoints):
            self.waypoints = waypoints

    def return_smooth_path(self):

        x_coords, y_coords = zip(*(self.waypoints))
        # Convert the resulting tuples into NumPy arrays
        original_x = np.array(x_coords)
        original_y = np.array(y_coords)
        # Calculate the cumulative distance 't' parameter
        distances = np.sqrt(np.diff(original_x)**2 + np.diff(original_y)**2)
        t = np.insert(np.cumsum(distances), 0, 0)
        
        # Create Cubic Spline interpolation functions for x(t) and y(t)
        cs_x = CubicSpline(t, original_x)
        cs_y = CubicSpline(t, original_y)
        
        # Generate 500 fine points for the smooth curve
        t_fine = np.linspace(t.min(), t.max(), 500)
        smooth_x = cs_x(t_fine)
        smooth_y = cs_y(t_fine)
        coords = np.array(list(zip(smooth_x, smooth_y)))
        return coords

if __name__ == "__main__":
     
    waypoints = [ (1, 2), (2, 4), (3, 4), (4, 5), (9, 7), (10, 5) ]
    cub = CubicSplinePath(waypoints)
    coordi = cub.return_smooth_path()
    
    plt.figure(figsize=(10, 7))

    plt.plot(coordi[:, 0], coordi[:, 1],
            linestyle='-', color='blue', 
            label='Smooth Path', linewidth=2)
    plt.title('Smooth Path Interpolation')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend()
    plt.axis('equal')
    plt.show()
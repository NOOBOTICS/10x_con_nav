import matplotlib.pyplot as plt
from smooth_path_cpy import SmoothPath
from cubic_spline_class import CubicSplinePath
import numpy as np

#-------------------add custom paths here ----------------------

tj = [(1,2) , (2,4), (3,4), (4,5), (9,7), (10, 5)]

#---------------------------------------------------------------

sm_path_custom = SmoothPath(tj)
sm_path_cubic = CubicSplinePath(tj)


# 1. Access the coordinate arrays from the class instance
path_custom_coords = sm_path_custom.return_smooth_path() 
path_cubic_coords = sm_path_cubic.return_smooth_path() 

ts_custom = np.array([x for x in range(0, len(path_custom_coords))])
ts_cubic = np.array([x for x in range(0, len(path_cubic_coords))])

path_custom_coords_ts = [(p[0], p[1], t) for p, t in zip(path_custom_coords, ts_custom)]
path_cubic_coords_ts = [(p[0], p[1], t) for p, t in zip(path_cubic_coords, ts_custom)]


print("Custom Smooth Path Coordinates:" ,path_custom_coords_ts)
print("Custom Smooth Path Coordinates:" ,path_cubic_coords_ts)



fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5)) 

# Plot on the first axis (left) using the extracted coordinates
ax1.plot(path_custom_coords[:, 0], path_custom_coords[:, 1], color='blue')
ax1.set_title('Custom Smooth Path')
ax1.scatter(*zip(*tj), color='k', s=50, label='Given Points') # Add given points for context
ax1.legend()
ax1.axis('equal')

# Plot on the second axis (right) using the extracted coordinates
ax2.plot(path_cubic_coords[:, 0], path_cubic_coords[:, 1], color='red')
ax2.set_title('Cubic Spline Path')
ax2.scatter(*zip(*tj), color='k', s=50, label='Given Points')
ax2.legend()
ax2.axis('equal')

plt.tight_layout()
plt.show()
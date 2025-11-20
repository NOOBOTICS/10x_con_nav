import numpy as np
import matplotlib.pyplot as plt

# The base class for calculating arc segments
class CircularArc:
    
    def __init__(self):
        """Initializes the CircularArc class."""
        self.tj = []
        pass

    def single_plot(self, segment_coords, color='blue', label='Trajectory', title='Combined Path Segments'):
        """Plots the coordinates of a single trajectory segment."""
        # Note: Points used for the plot title are updated to the user's new list
        points = np.array(self.tj)
        

        plt.figure(figsize=(8, 8))
        
        # Plot the trajectory
        plt.plot(segment_coords[:, 0], segment_coords[:, 1],
                 color=color, linewidth=3, label=label)

        # Plot the given points
        plt.scatter(points[:, 0], points[:, 1],
                    color='red', s=100, zorder=5, label='Given Path Points')
        
        plt.title(title)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.axis('equal') 
        plt.legend()
        plt.grid(True)
        plt.show()

    def _get_shortest_arc_angle(self, start_angle, end_angle):
        """
        Calculates the signed shortest angular difference between two angles.
        Result is always between (-pi, pi].
        """
        diff = end_angle - start_angle
        # Normalize diff to [-pi, pi]
        if diff > np.pi:
            diff -= 2 * np.pi
        elif diff <= -np.pi:
            diff += 2 * np.pi
        return diff

    def find_circle_center_radius(self, p1, p2, p3):
        """
        Finds the center (h, k) and radius (r) of the circle passing through
        three non-collinear points p1, p2, and p3. Returns None, None, None if collinear.
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        # Calculate the denominator (D)
        D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))

        # Check for collinearity
        if np.isclose(D, 0):
            return None, None, None

        # Calculate the center (h, k) using the standard formula
        h = ( (x1**2 + y1**2) * (y2 - y3) +
            (x2**2 + y2**2) * (y3 - y1) +
            (x3**2 + y3**2) * (y1 - y2) ) / D

        k = ( (x1**2 + y1**2) * (x3 - x2) +
            (x2**2 + y2**2) * (x1 - x3) +
            (x3**2 + y3**2) * (x2 - x1) ) / D

        # Calculate the radius (r)
        r = np.sqrt((x1 - h)**2 + (y1 - k)**2)

        return h, k, r


    def generate_arc_segments(self,p1, p2, p3, num_points=200):
        """
        Generates two sets of coordinates for the curve segments: P1->P2 and P2->P3,
        always selecting the shortest arc.
        Returns two 2D numpy arrays.
        """
        center_x , center_y, radius = self.find_circle_center_radius(p1, p2, p3)
        seg_points = max(2, num_points // 2)

        if center_x is None:
        # --- Collinear Case (Straight Line Segments) ---
            x1, y1 = p1
            x2, y2 = p2
            x3, y3 = p3

            # Segment 1: P1 to P2
            x_line_1 = np.linspace(x1, x2, seg_points, endpoint=True)
            y_line_1 = np.linspace(y1, y2, seg_points, endpoint=True)
            seg1_coords = np.vstack((x_line_1, y_line_1)).T

            # Segment 2: P2 to P3
            x_line_2 = np.linspace(x2, x3, seg_points, endpoint=True)
            y_line_2 = np.linspace(y2, y3, seg_points, endpoint=True)
            seg2_coords = np.vstack((x_line_2, y_line_2)).T
            
            return seg1_coords, seg2_coords

        # --- Circular Arc Logic (if not collinear) ---
        
        # 1. Calculate the angle of each point relative to the center 
        angle1 = np.arctan2(p1[1] - center_y, p1[0] - center_x)
        angle2 = np.arctan2(p2[1] - center_y, p2[0] - center_x)
        angle3 = np.arctan2(p3[1] - center_y, p3[0] - center_x)
        
        start_angle_1 = angle1
        angle_diff_1 = self._get_shortest_arc_angle(angle1, angle2)
        end_angle_1 = start_angle_1 + angle_diff_1
        
        arc_angles_1 = np.linspace(start_angle_1, end_angle_1, seg_points, endpoint=True)
        x_arc_1 = center_x + radius * np.cos(arc_angles_1)
        y_arc_1 = center_y + radius * np.sin(arc_angles_1)
        seg1_coords = np.vstack((x_arc_1, y_arc_1)).T
        
        start_angle_2 = angle2
        angle_diff_2 = self._get_shortest_arc_angle(angle2, angle3)
        end_angle_2 = start_angle_2 + angle_diff_2
        
        arc_angles_2 = np.linspace(start_angle_2, end_angle_2, seg_points, endpoint=True)
        x_arc_2 = center_x + radius * np.cos(arc_angles_2)
        y_arc_2 = center_y + radius * np.sin(arc_angles_2)
        seg2_coords = np.vstack((x_arc_2, y_arc_2)).T

        return seg1_coords, seg2_coords
    

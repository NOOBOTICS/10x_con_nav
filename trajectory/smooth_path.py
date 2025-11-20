import numpy as np
from circular_arc_class import CircularArc


class SmoothPath:

    # import tensorflow_datasets as tfds
    def __init__(self, trajectory_points):
        self.trajectory_points= trajectory_points
        self.seglst = []
        self.temp = np.array([])
        self.seg_class = CircularArc()
        self.seg_class.tj = trajectory_points  # Pass the trajectory points to CircularArc instance
        
        
    def return_smooth_path(self):

        for i in range(len(self.trajectory_points) - 2):
            p1 = self.trajectory_points[i]
            p2 = self.trajectory_points[i + 1]
            p3 = self.trajectory_points[i + 2]
            
            seg1 , seg2  = self.seg_class.generate_arc_segments(p1, p2, p3 , num_points=50) # each segment will have 100 points (0, 99) indexes
            
            # print(f"iteration {i}: {seg1}  and {seg2}" )
            
            if i == 0:
                self.seglst.append(seg1)
                self.seglst.append(seg2)
                # print("First iteration, adding both segments.", self.seglst)
            else:
                self.temp = (seg1 + self.seglst.pop())/2  # takes the averate of the duplicate segments
                self.seglst.append(self.temp)
                self.seglst.append(seg2) 
                # print(f"Iteration {i}, merging segments. Current seglst:", self.seglst)

 
        final_segment = np.concatenate(self.seglst)
        # print("Final smooth path shape:", final_segment)
        return final_segment
    
        # self.seg_class.single_plot(final_segment, color='blue', label='Smooth Trajectory')
if __name__ == "__main__":

    tj = [(1,2) , (2,4), (3,4), (4,5), (9,7), (10, 5)]


    sm_path = SmoothPath(tj)

    path_sm = sm_path.return_smooth_path()

    sm_path.seg_class.single_plot(path_sm)

            
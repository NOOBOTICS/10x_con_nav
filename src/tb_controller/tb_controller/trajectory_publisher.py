import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from trajectory.cubic_spline_class import CubicSplinePath
from trajectory.smooth_path import SmoothPath

# Your trajectory data (assuming it's a list of (x, y) list)

# path2 = [
#     [-1.33, -2.47], # first end point
#     [0.0, 0.0], 
#     [5.0, -2.47], #second end point
#     [4.0, 0], 
#     [5.0, 3.5],  #third end point
#     [ 1.33, 2.5],
#     [ -1.33, 3.5], #fourth end point
#     # [0.0, 0.0]
# ]

path = [
        [0.7,-1.3], [3,-1.3], [4,0.5], [3,2.2], [1,2.2]
         ]

smp = SmoothPath(path)
cup = CubicSplinePath(path)

YOUR_TRAJECTORY_POINTS = smp.return_smooth_path()

# YOUR_TRAJECTORY_POINTS = cup.return_smooth_path()

# YOUR_TRAJECTORY_POINTS = np.array(path)


class PathPublisher(Node):
    def __init__(self, trajectory_coords):
        super().__init__('trajectory_publisher')
        
        # Publisher for the Path message
        self.path_publisher = self.create_publisher(Path, 'custom_trajectory', 10)
        
        self.trajectory_coords = trajectory_coords
        self.timer = self.create_timer(1.0, self.publish_path) # Publish at 1 Hz

        self.get_logger().info('Trajectory Publisher started.')

    def publish_path(self):
        path_msg = Path()
        
        # 1. Set the Header: IMPORTANT for RViz visualization!
        # Use the same frame_id as your map (usually 'map')
        path_msg.header.frame_id = 'map' 
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        poses = []
        for x, y in self.trajectory_coords:
            pose = PoseStamped()
            
            # 2. Set the PoseStamped Header
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            
            # 3. Set the Position (Assuming 2D, Z=0)
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # 4. Set the Orientation (usually Identity Quaternion for a simple path)
            pose.pose.orientation.w = 1.0 # (x=0, y=0, z=0, w=1 is identity/no rotation)
            
            poses.append(pose)
        
        path_msg.poses = poses
        self.path_publisher.publish(path_msg)
        # self.get_logger().info('Publishing Path message...')

def main(args=None):
    rclpy.init(args=args)
    # NOTE: You would typically integrate your sm_path_cubic.path here
    publisher_node = PathPublisher(YOUR_TRAJECTORY_POINTS)
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
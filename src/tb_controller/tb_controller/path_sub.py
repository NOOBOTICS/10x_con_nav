import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathSubscriber(Node):
    """
    Subscribes to a Path topic (representing a trajectory) and reports path information.
    """
    def __init__(self):
        super().__init__('path_subscriber_node')
        self.subscription = self.create_subscription(
            Path,
            '/custom_trajectory', 
            self.listener_callback,
            1)
        self.get_logger().info('Path Subscriber Node started. Listening to /custom_trajectory.')

    def listener_callback(self, msg):
        num_points = len(msg.poses)
        
        if num_points > 0:
            # Optionally log the start point
            start_x = msg.poses[0].pose.position.x
            start_y = msg.poses[0].pose.position.y
            
            self.get_logger().info(
                f'Received new path with {num_points} waypoints. Starting at x={start_x:.2f}, y={start_y:.2f}'
            )
        else:
            self.get_logger().warn('Received an empty path.')

def main(args=None):
    rclpy.init(args=args)
    path_subscriber = PathSubscriber()
    rclpy.spin(path_subscriber)
    path_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
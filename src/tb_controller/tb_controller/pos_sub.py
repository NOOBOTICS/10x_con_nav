import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped # Confirmed type for /amcl_pose
from tf_transformations import euler_from_quaternion

class PoseSubscriber(Node):
    """
    Subscribes to the /amcl_pose topic and logs the robot's current position and orientation.
    """
    def __init__(self):
        super().__init__('pose_subscriber_node')
        
        # Subscriber for the estimated robot pose
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.get_logger().info('Pose Subscriber Node started. Listening to /amcl_pose.')

    def listener_callback(self, msg):
        """
        Callback function executed every time a new message is received on /amcl_pose.
        """
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation and convert to Yaw (for easy viewing)
        q = msg.pose.pose.orientation
        # Note: euler_from_quaternion requires the arguments in a list/tuple
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.get_logger().info(
            f'Robot Pose: x={x:.2f}, y={y:.2f}, yaw_rad={yaw:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    
    try:
        rclpy.spin(pose_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        pose_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
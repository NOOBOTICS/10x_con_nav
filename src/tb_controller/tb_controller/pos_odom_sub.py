import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np # Imported for euler_from_quaternion, though not strictly necessary for this simple script

class SimpleOdometrySubscriber(Node):
    
    def __init__(self):
        super().__init__('simple_odometry_subscriber') 
        
        self.subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback,
            10)
        self.get_logger().info('Simple Odometry Subscriber started. Listening to /odom.')

    def odom_callback(self, msg):
       
        # --- 1. Coordinates (Position) ---
        x = msg.pose.pose.position.x + 2
        y = msg.pose.pose.position.y + 0.5
        
        # --- 2. Orientation (Convert Quaternion to Yaw Angle) ---
        q = msg.pose.pose.orientation
        # Unpack quaternion to get roll, pitch, and yaw (we only care about yaw)
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # --- 3. Velocities (Linear and Angular) ---
        linear_vel_x = msg.twist.twist.linear.x
        angular_vel_z = msg.twist.twist.angular.z
        
        # Logging data at the INFO level
        self.get_logger().info(
            f'POSE: (x={x:.2f}, y={y:.2f}, Yaw={yaw:.2f} rad) | '
            f'VELOCITY: (Linear X={linear_vel_x:.2f} m/s, Angular Z={angular_vel_z:.2f} rad/s)'
        )

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = SimpleOdometrySubscriber() 
    
    try:
        rclpy.spin(odom_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        odom_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
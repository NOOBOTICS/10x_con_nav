import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    """
    Publishes a constant forward linear velocity to the robot's command velocity topic.
    """
    def __init__(self):
        super().__init__('velocity_publisher_node')
        
        # Publisher for sending velocity commands#
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer to publish velocity at a rate of 10 Hz (10 times per second)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.linear_speed =-0.0


        self.get_logger().info('Velocity Publisher Node started. Publishing constant forward velocity.')

    def timer_callback(self):
        """
        Called periodically by the timer to publish the Twist message.
        """
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = 0.0
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing linear velocity: {self.linear_speed:.1f} m/s')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot when shutting down
        stop_msg = Twist()
        velocity_publisher.publisher_.publish(stop_msg)
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
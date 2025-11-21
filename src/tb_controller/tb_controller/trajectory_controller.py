import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
from nav_msgs.msg import Path


class TargetPositionController(Node):
    
    def __init__(self):
        super().__init__('target_position_controller') 

        self.i = 0
        self.num_points = 0 

        # --- Parameters ---
        self.target_x = 1.0  # meters
        self.target_y = -1.0 # meters
        
        self.get_logger().info(f'🚀 Target set to: ({self.target_x}, {self.target_y})')

        # Proportional Gains (Tuning parameters for the controller)
        self.kp_linear = 0.5  # Gain for linear velocity (proportional to distance error)
        self.kp_angular = 1.0 # Gain for angular velocity (proportional to heading error)
        
        # Thresholds
        self.distance_threshold = 0.1 # meters (stop when this close to target)
        self.angular_threshold = 0.05 # radians (minimum error to apply angular velocity)

        # --- ROS 2 Publishers and Subscribers ---
        # Publisher for sending velocity commands
        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber for receiving Odometry
        self.odom_subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback,
            10)
        
        self.subscription = self.create_subscription(
            Path,
            '/custom_trajectory', 
            self.listener_callback,
            1)
        self.get_logger().info('Path Subscriber Node started. Listening to /custom_trajectory.')
            
        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Flag to indicate if the goal is reached
        self.goal_reached = False

    def listener_callback(self, msg):
        self.path = msg     
        self.num_points = len(msg.poses)

        
        if self.num_points > 0:
            # Optionally log the start point
            self.target_x = msg.poses[self.i].pose.position.x
            self.target_y = msg.poses[self.i].pose.position.y
            
            self.get_logger().info(
                f'Received new path with {self.num_points} waypoints. Starting at x={self.target_x:.2f}, y={self.target_y:.2f}'
            )
        else:
            self.get_logger().warn('Received an empty path.')

    def odom_callback(self, msg):
        """
        Processes odometry data and calculates/publishes the control commands.
        """
        

        # 1. Update Current State
        self.current_x = msg.pose.pose.position.x + 2.0  # as per adjustment
        self.current_y = msg.pose.pose.position.y + 0.5
        
        q = msg.pose.pose.orientation
        # Convert quaternion to Yaw angle
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 2. Calculate Errors
        delta_x = self.target_x - self.current_x
        delta_y = self.target_y - self.current_y
        
        distance_to_target = math.sqrt(delta_x**2 + delta_y**2)
        
        # 3. Check for Goal Reached
        if distance_to_target < self.distance_threshold:
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info(' Goal Reached!')
        

        # 4. Calculate Required Heading
        # The angle from the robot's current position to the target
        angle_to_target = math.atan2(delta_y, delta_x)
        
        # Heading error
        angular_error = angle_to_target - self.current_yaw
        
        # Normalize the error to the range (-pi, pi]
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # 5. Calculate Control Output (Velocities)
        twist_msg = Twist()


        # if abs(angular_error) > self.angular_threshold:
            # First, prioritize turning to face the target
        # twist_msg.linear.x = 0.0

        twist_msg.angular.z = self.kp_angular * angular_error
        # Clamp angular velocity to a maximum value
        twist_msg.angular.z = max(min(twist_msg.angular.z, 1.8), -1.8)
        self.get_logger().info(f'goal pos is ={self.target_x},{self.target_y}')
        
        self.get_logger().info(f'Turning: Error={angular_error:.2f}, Angular Vel={twist_msg.angular.z:.2f}')
        

        # else:
            # Once facing the target, move forward
        twist_msg.linear.x = self.kp_linear * distance_to_target
        # twist_msg.angular.z = 0.0
        
        # Clamp linear velocity to a maximum value
        twist_msg.linear.x = max(min(twist_msg.linear.x, 0.26), -0.26) 

        self.get_logger().info(f'Moving: Dist={distance_to_target:.2f}, Linear Vel={twist_msg.linear.x:.2f}')

        # 6. Publish Command
        self.vel_publisher_.publish(twist_msg)


    def stop_robot(self):
        # If no path received yet
        if self.num_points == 0:
            self.get_logger().info('No path received yet. Cannot proceed to next waypoint.')
            return

        # If last waypoint reached → stop completely
        if self.i == self.num_points - 1:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.vel_publisher_.publish(twist_msg)
            self.get_logger().info('Robot Stopped. Final waypoint reached.')
        else:
            # Move to next waypoint WITHOUT stopping the robot
            self.i += 1
            self.goal_reached = False
            
            # Update target
            self.target_x = self.path.poses[self.i].pose.position.x
            self.target_y = self.path.poses[self.i].pose.position.y
            self.get_logger().info(f'Moving to waypoint {self.i}: ({self.target_x}, {self.target_y})')


        
def main(args=None):
    rclpy.init(args=args)
    controller = TargetPositionController()
    
    rclpy.spin(controller)
    controller.get_logger().info('Controller stopped by user.')
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose as TurtlePose
import math

# Simplified class name
class turtle_control_node(Node):

    def __init__(self):
        super().__init__('turtle_control_marcus')

        # Initializing goal (can be customizable)
        self.goal = Pose2D(x=5.54, y=5.54)

        # Defining control parameters
        self.distance_threshold = 0.05
        self.angle_threshold = 0.01
        self.max_angular_velocity = 1.0
        self.max_linear_velocity = 1.0

        # Creating publisher and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(TurtlePose, '/turtle1/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)

        # Initializing state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.control_loop()

    def goal_callback(self, msg):
        self.goal = msg

    def control_loop(self):
        # Calculate distance and angle error
        distance_error = math.sqrt(((self.goal.x - self.x) ** 2) + ((self.goal.y - self.y) ** 2))
        angle_error = math.atan2(self.goal.y - self.y, self.goal.x - self.x) - self.theta

        # Check if reached goal
        if distance_error < self.distance_threshold:
            self.publish_command(0.0, 0.0)
            return

        # Adjust angular velocity based on angle error
        if abs(angle_error) > self.angle_threshold:
            angular_velocity = self.max_angular_velocity * math.copysign(1.0, angle_error)
        else:
            angular_velocity = 0.0

        # Adjust linear velocity based on distance error
        linear_velocity = self.max_linear_velocity * math.tanh(distance_error)

        # Publish velocity command
        self.publish_command(linear_velocity, angular_velocity)

    def publish_command(self, linear_velocity, angular_velocity):
        command = Twist()
        command.linear.x = linear_velocity
        command.angular.z = angular_velocity
        self.cmd_vel_pub.publish(command)

def main(args=None):
    rclpy.init(args=args)
    navigator = turtle_control_node()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ackermann_msgs.msg import AckermannDriveStamped

class Simulate(Node):
    """
    Broadcasts a static transform from world>odom frame at startup.
    Combines steering command and drive command into AckermannSteering message for simulation.
    """

    def __init__(self):
        super().__init__('simulate')

        # Declare parameters
        self.declare_parameter("rate", 200)
        self.declare_parameter("wheel_radius", 0.054)
        self.declare_parameter("gear_ratio", 5.)
        self.declare_parameter("max_rpm", 3.7*3*1450)
        self.declare_parameter("steer_angle_range", 1.0)
        self.declare_parameter("steer_left_max", 1700)
        self.declare_parameter("steer_right_max", 1300)
        self.declare_parameter("cmd_max", 2000)
        self.declare_parameter("cmd_min", 1000)
        
        self.loop_rate = (self.get_parameter('rate').get_parameter_value().integer_value)
        self.wheel_radius = (self.get_parameter('wheel_radius').get_parameter_value().double_value)
        self.gear_ratio = (self.get_parameter('gear_ratio').get_parameter_value().double_value)
        self.max_rpm = (self.get_parameter('max_rpm').get_parameter_value().double_value)
        self.steer_angle_range = (self.get_parameter('steer_angle_range').get_parameter_value().double_value)
        self.steer_left_max = (self.get_parameter('steer_left_max').get_parameter_value().integer_value)
        self.steer_right_max = (self.get_parameter('steer_right_max').get_parameter_value().integer_value)
        self.cmd_max = (self.get_parameter('cmd_max').get_parameter_value().integer_value)
        self.cmd_min = (self.get_parameter('cmd_min').get_parameter_value().integer_value)

        # Define other variables
        self.drive_neutral = (self.cmd_max + self.cmd_min) / 2
        self.steering_neutral = (self.steer_left_max + self.steer_right_max) / 2
        self.drive_cmd = self.drive_neutral
        self.steering_cmd = self.steering_neutral

        # Subscribers
        self.drive_cmd_sub = self.create_subscription(Int32, 'drive_cmd', self.drive_cmd_callback, 10)
        self.steering_cmd_sub = self.create_subscription(Int32, 'steering_cmd', self.steering_cmd_callback, 10)

        # Publishers
        self.ackermann_cmd_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)

        # Create timer
        self.timer_step = 1/self.loop_rate
        self.timer = self.create_timer(self.timer_step, self.timer_callback)


    def timer_callback(self):
        """
        Description: Main timer callback function, publishes ackermann messages
            - Check robot state
            - If MOVING, etermine distance to target and if target has been reached and if not at target, move toward target
            - If STOPPED, then stop moving
            - Update poses and call frame update function to transform frames
        """
        # Create ackerman message 
        ackermann_cmd = AckermannDriveStamped()

        # Convert command messages to robot commands using robot data
        speed_factor = (self.max_rpm * 2 * np.pi * self.wheel_radius / (60 * self.gear_ratio)) / ((self.cmd_max - self.cmd_min) / 2)
        steering_factor = self.steer_angle_range / np.absolute(self.steer_left_max - self.steer_right_max)
        speed = speed_factor * (self.drive_cmd - self.drive_neutral)
        steering_angle = steering_factor * (self.steering_cmd - self.steering_neutral)

        # Add values to message
        ackermann_cmd.drive.speed = speed
        ackermann_cmd.drive.steering_angle = steering_angle

        # Add time and publish message
        time = self.get_clock().now().to_msg()
        ackermann_cmd.header.stamp = time
        self.ackermann_cmd_pub.publish(ackermann_cmd)

    
    def drive_cmd_callback(self, msg):
        """
        Description: Callback for drive_cmd messages, stores data
        """
        self.drive_cmd = msg.data

    
    def steering_cmd_callback(self, msg):
        """
        Description: Callback for steering_cmd messages, stores data
        """
        self.steering_cmd = msg.data


def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    node = Simulate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
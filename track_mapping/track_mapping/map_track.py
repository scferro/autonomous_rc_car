"""
Publishes goal_poses for the nubot that will cause it to explore the environment. When the robot gets close to a wall, it will turn

PUBLISHERS:
  + goal_pose (PoseStamped) - The goal pose of the robot

SUBSCRIBERS:
  + scan (LaserScan) - The laser scan data from the lidar

"""
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped, Transform, Vector3, Twist
from std_msgs.msg import Header
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from enum import Enum, auto
from tf2_ros import Buffer, TransformListener

class State(Enum):
    MOVING = auto(),

class Explore(Node):
    def __init__(self):
        """
        Initializes the Explore node
        """
        super().__init__('explore')

        # Initiate Variables
        self.state = State.MOVING

        # Create tf listener
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)

        # Create cmd_vel publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create laser scan subscriber
        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            10)
        
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        '''
        The callback loop for the main timer.
        '''

        # Check if there is an obstacle - if there is, turn, if not, go straight
        self.move_forward()
        

    def laser_scan_callback(self, msg):
        pass

    def move_forward(self):
        '''
        publishes goal poses to move the robot straight ahead
        '''

        self.get_logger().info('Moving forward...')

        twist_linear = Vector3(x=0.5, y=0.0, z=0.0)
        twist_angular = Vector3(x=0.0, y=0.0, z=1.0)

        twist_command = Twist(linear=twist_linear, angular=twist_angular)

        self.cmd_vel_publisher.publish(twist_command)

def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
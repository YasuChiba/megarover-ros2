import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class GlobalLocalization(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')


        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        

def main():
    rclpy.init()
    node = GlobalLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
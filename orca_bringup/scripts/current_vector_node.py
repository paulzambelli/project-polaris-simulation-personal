#!/usr/bin/env python3

import time
from urllib import request

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3d
import math



class CurrentVector(Node):

    def __init__(self):
        super().__init__('current_vector')
        self.publisher_ = self.create_publisher(Vector3d, 'topic', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Vector3d()
        msg.data = [1, 1, 1]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
            


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CurrentVector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

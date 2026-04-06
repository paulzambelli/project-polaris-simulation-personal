#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

# ros2 run orca_bringup current_vector_node.py


class CurrentVector(Node):

    def __init__(self):
        super().__init__('current_vector')
        self.publisher_ = self.create_publisher(Vector3, '/ocean_current', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Vector3()
        msg.x = 1.0
        msg.y = 1.0
        msg.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: x=%s y=%s z=%s' % (msg.x, msg.y, msg.z))


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

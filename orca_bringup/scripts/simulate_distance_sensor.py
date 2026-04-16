#!/usr/bin/env python3

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

class DistancePublisherNode(Node):

    def __init__(self):
        super().__init__('distance_simulator')

        self.publisher_ = self.create_publisher(Float32, '/top/ultrasonic/distance', qos_profile_sensor_data)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.pos_z = -50.0

        # Use wall clock for the timer: with use_sim_time, the default ROS clock tracks
        # /clock. If sim time does not advance (no clock yet, paused sim), no publishes
        # would occur and ice checks never see /top/ultrasonic/distance.
        timer_period = 0.5
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

    def timer_callback(self):
        msg = Float32()
        diff = -0.7 - self.pos_z # assuming the ice sheet is at -0.7 meters
        msg.data = diff
        self.publisher_.publish(msg)

    def _on_odom(self, msg: Odometry):
        self.pos_z = msg.pose.pose.position.z
        
        
def main():
    rclpy.init()
    node = DistancePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
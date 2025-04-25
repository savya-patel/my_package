#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class FakeRPLidar(Node):
    def __init__(self):
        super().__init__('fake_rplidar')
        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        # 360° scan: 360 readings at 1° increments
        self.num_readings = 360
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'       # or your lidar frame
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = 2.0 * math.pi / self.num_readings
        scan.time_increment = (1.0 / 10.0) / self.num_readings
        scan.range_min = 0.15
        scan.range_max = 6.0

        # Here we simulate a constant 2 m range in every direction.
        # You can replace this with whatever pattern you like,
        # e.g. a moving wall (sinusoid), random noise, or reading a map.
        scan.ranges = [2.0 for _ in range(self.num_readings)]
        scan.intensities = [100.0 for _ in range(self.num_readings)]

        self.pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = FakeRPLidar()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


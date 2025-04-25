#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

class WorldMarkerPublisher(Node):
    def __init__(self):
        super().__init__('world_marker_publisher')
        self.pub = self.create_publisher(MarkerArray, 'world_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

        # define your obstacles here: (x, y, radius, height)
        self.obstacles = [
            (2.0,  1.0, 0.2, 0.5),   # cone‐like cylinder
            (1.5, -0.5, 0.15, 0.4),
            (4.0,  2.5, 0.25, 0.6),
            (7.0,  5.0, 1.0, 0.1),   # a flat wall segment
        ]

    def publish_markers(self):
        ma = MarkerArray()
        for i, (x, y, r, h) in enumerate(self.obstacles):
            m = Marker()
            m.header = Header(stamp=self.get_clock().now().to_msg(),
                              frame_id='map')  
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            # position it so the base sits on z=0
            m.pose = Pose()
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = h/2.0
            m.pose.orientation = Quaternion(x=0,y=0,z=0,w=1)
            # size: diameter = 2*r, height = h
            m.scale = Vector3(x=2*r, y=2*r, z=h)
            m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            ma.markers.append(m)
        self.pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = WorldMarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

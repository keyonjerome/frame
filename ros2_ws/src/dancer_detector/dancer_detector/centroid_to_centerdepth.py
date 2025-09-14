#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from example_interfaces.msg import Float32MultiArray

class CentroidToCenterDepth(Node):
    def __init__(self):
        super().__init__('centroid_to_centerdepth')
        in_topic  = self.declare_parameter('in_topic', 'person_centroid').value
        out_topic = self.declare_parameter('out_topic', '/person_center_depth').value
        self.sub = self.create_subscription(PointStamped, in_topic, self.cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, out_topic, 10)

    def cb(self, msg: PointStamped):
        out = Float32MultiArray()
        # Interpret point.x/y as pixel coordinates, point.z as depth (m)
        out.data = [float(msg.point.x), float(msg.point.z)]
        self.pub.publish(out)

def main():
    rclpy.init(); n = CentroidToCenterDepth()
    try: rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

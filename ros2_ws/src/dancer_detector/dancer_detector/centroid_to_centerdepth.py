#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from example_interfaces.msg import Float32MultiArray
from std_srvs.srv import Trigger

class CentroidGateToCenterDepth(Node):
    def __init__(self):
        super().__init__('centroid_gate_to_centerdepth')

        # Topics & initial active state
        self.in_topic  = self.declare_parameter('in_topic', 'person_centroid').value
        self.out_topic = self.declare_parameter('out_topic', '/person_center_depth').value
        self.active    = bool(self.declare_parameter('initial_active', False).value)

        # I/O
        self.sub = self.create_subscription(PointStamped, self.in_topic, self.cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, self.out_topic, 10)

        # Services: /dancer_detector/start, /dancer_detector/stop
        self.srv_start = self.create_service(Trigger, '/dancer_detector/start', self.on_start)
        self.srv_stop  = self.create_service(Trigger, '/dancer_detector/stop',  self.on_stop)

        self.get_logger().info(f'Gate ready. active={self.active}  in="{self.in_topic}"  out="{self.out_topic}"')

    def on_start(self, req, resp):
        self.active = True
        resp.success = True
        resp.message = 'gate enabled'
        self.get_logger().info('Gate ENABLED')
        return resp

    def on_stop(self, req, resp):
        self.active = False
        resp.success = True
        resp.message = 'gate disabled'
        self.get_logger().info('Gate DISABLED')
        return resp

    def cb(self, msg: PointStamped):
        if not self.active:
            return
        out = Float32MultiArray()
        # x = pixel u; z = depth (m)
        out.data = [float(msg.point.x), float(msg.point.z)]
        self.pub.publish(out)

def main():
    rclpy.init()
    n = CentroidGateToCenterDepth()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class GnssNode(Node):
    def __init__(self):
        super().__init__('gnss_node')
        self.pub = self.create_publisher(String, '/gnss', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        lat = 34.0 + (random.random()-0.5)*0.001
        lon = -117.0 + (random.random()-0.5)*0.001
        msg = String()
        msg.data = f'{lat:.6f},{lon:.6f}'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GnssNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.pub = self.create_publisher(String, '/vision/targets', 10)
        self.timer = self.create_timer(2.0, self.timer_cb)

    def timer_cb(self):
        # publish a simulated detection: "color,x,y"
        msg = String()
        msg.data = 'red,2.0,3.0'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


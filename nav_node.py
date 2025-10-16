import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute
import time

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.mode_sub = self.create_subscription(String, '/mission_mode', self.mode_cb, 10)
        self.gnss_sub = self.create_subscription(String, '/gnss', self.gnss_cb, 10)
        self.vision_sub = self.create_subscription(String, '/vision/targets', self.vision_cb, 10)
        
        
        self.declare_parameter('waypoints', [(2.0,2.0),(5.0,5.0),(8.0,4.0),(3.0,7.0)])
        self.waypoints = self.get_parameter('waypoints').get_parameter_value().string_value if False else None
        self.wpoints = [(2.0,2.0),(5.0,5.0),(8.0,4.0),(3.0,7.0)]
        self.current_mode = 'manual'
        self.client = self.create_client(TeleportAbsolute,'/turtle1/teleport_absolute')

    def mode_cb(self, msg: String):
        prev = self.current_mode
        self.current_mode = msg.data
        if self.current_mode == 'autonomy':
            self.get_logger().info('Starting autonomous waypoint navigation')
            self.visit_waypoints()

    def gnss_cb(self, msg: String):
        # could be used for logging
        pass

    def vision_cb(self, msg: String):
        # could be used to trigger vision target visits
        pass

    def call_teleport(self, x,y,theta=0.0):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def visit_waypoints(self):
        self.get_logger().info('[LED] Red=Autonomous')
        for idx,(x,y) in enumerate(self.wpoints):
            if self.current_mode != 'autonomy':
                self.get_logger().info('Autonomy aborted by mode change')
                return
            self.get_logger().info(f'Visiting waypoint {idx+1}: ({x},{y})')
            self.call_teleport(x,y)
            self.get_logger().info('[LED] Green=Target Reached')
            time.sleep(1.0)
        self.get_logger().info('Completed waypoint list')

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


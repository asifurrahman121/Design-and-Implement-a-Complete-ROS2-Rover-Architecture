import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_action_interfaces.action import Deliver
from turtlesim.srv import TeleportAbsolute
import time

class DeliveryActionServer(Node):
    def __init__(self):
        super().__init__('delivery_action_server')
        self._action_server = ActionServer(
            self,
            Deliver,
            'deliver_item',
            self.execute_callback)
        self.client = self.create_client(TeleportAbsolute,'/turtle1/teleport_absolute')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: {goal_handle.request.item} to ({goal_handle.request.x},{goal_handle.request.y})')
        feedback = Deliver.Feedback()
        # simulate pick up at current location
        feedback.status = 'Picking up'
        feedback.progress = 10.0
        goal_handle.publish_feedback(feedback)
        time.sleep(1.0)
        # move to dropoff
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Teleport service not available')
            goal_handle.abort()
            res = Deliver.Result()
            res.success = False
            res.message = 'teleport missing'
            return res
        # teleport to pick location
        # teleport to dropoff
        req = TeleportAbsolute.Request()
        req.x = float(goal_handle.request.x)
        req.y = float(goal_handle.request.y)
        req.theta = 0.0
        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        notime = 0
        for p in [30,60,100]:
            feedback.status = 'Delivering'
            feedback.progress = float(p)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)
        result = Deliver.Result()
        result.success = True
        result.message = 'Delivered'
        self.get_logger().info('Delivery complete')
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


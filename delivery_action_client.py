import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_action_interfaces.action import Deliver

class DeliveryClient(Node):
    def __init__(self):
        super().__init__('delivery_action_client')
        self._client = ActionClient(self, Deliver, 'deliver_item')

    def send_goal(self, item, x, y):
        goal_msg = Deliver.Goal()
        goal_msg.item = item
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        self._client.wait_for_server()
        self._future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, self._future)
        goal_handle = self._future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self._result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._result_future)
        result = self._result_future.result().result
        self.get_logger().info(f'Result: success={result.success}, message="{result.message}"')

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {fb.status} {fb.progress}%')

def main(args=None):
    rclpy.init(args=args)
    client = DeliveryClient()
    client.send_goal('sample_box', 8.0, 5.0)
    client.destroy_node()
    rclpy.shutdown()


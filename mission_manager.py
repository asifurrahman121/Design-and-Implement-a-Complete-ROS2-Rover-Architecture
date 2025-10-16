import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        self.declare_parameter('mission_mode', 'autonomy')
        self.pub = self.create_publisher(String, '/mission_mode', 10)

        self.add_on_set_parameters_callback(self.param_callback)
        
        # Get the initial value of the parameter
        mode = self.get_parameter('mission_mode').value
        self.publish_mode(mode)
        self.get_logger().info(f'Initial mission_mode: {mode}')

    def param_callback(self, params):
        for p in params:
            if p.name == 'mission_mode':  
                val = p.value
                self.publish_mode(val)
                self.get_logger().info(f'mission_mode changed to: {val}')
        return SetParametersResult(successful=True)  

    def publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


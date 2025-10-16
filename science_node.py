import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute
import cv2
import os
from datetime import datetime
import time

class ScienceNode(Node):
    def __init__(self):
        super().__init__('science_node')
        self.mode_sub = self.create_subscription(String, '/mission_mode', self.mode_cb, 10)
        self.gnss_sub = self.create_subscription(String, '/gnss', self.gnss_cb, 10)
        self.client = self.create_client(TeleportAbsolute,'/turtle1/teleport_absolute')
        self.latest_gnss = 'unknown'
        os.makedirs(os.path.expanduser('~/urc_logs'), exist_ok=True)

    def mode_cb(self, msg):
        if msg.data == 'science':
            x,y = 5.5,5.5
            self.get_logger().info(f'Moving to science site ({x},{y})')
            self.teleport(x,y)
            time.sleep(0.5)
            filename = self.capture_image()
            
            with open(os.path.expanduser('~/urc_logs/site_log.txt'), 'a') as f:
                f.write(f'{datetime.now().isoformat()} {self.latest_gnss} {filename}\n')
            self.get_logger().info(f'Saved image {filename} and logged GNSS')

    def gnss_cb(self, msg):
        self.latest_gnss = msg.data

    def teleport(self, x,y,theta=0.0):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def capture_image(self):
        # webcam, else dummy image
        path = os.path.expanduser('~/urc_logs')
        fname = os.path.join(path, f'site_{int(time.time())}.jpg')
        try:
            cap = cv2.VideoCapture(0)
            if cap is None or not cap.isOpened():
                raise RuntimeError('No camera')
            ret, frame = cap.read()
            cap.release()
            if ret:
                cv2.imwrite(fname, frame)
                return fname
            raise RuntimeError('camera read failed')
        except Exception as e:
            # dummy image
            import numpy as np
            img = (255 * np.ones((240,320,3), dtype='uint8'))
            cv2.putText(img, 'DUMMY', (50,120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.imwrite(fname, img)
            return fname

def main(args=None):
    rclpy.init(args=args)
    node = ScienceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


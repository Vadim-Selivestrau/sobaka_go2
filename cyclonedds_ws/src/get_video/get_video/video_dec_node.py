import rclpy
from rclpy.node import Node
from unitree_go.msg import Go2FrontVideoData
import cv2
import numpy as np

class FrontVideoSubscriber(Node):
    def __init__(self):
        super().__init__('front_video_subscriber')
        self.subscription = self.create_subscription(
            Go2FrontVideoData,
            '/frontvideostream',
            self.listener_callback,
            1)
        self.subscription  

    def listener_callback(self, msg: Go2FrontVideoData):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = FrontVideoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
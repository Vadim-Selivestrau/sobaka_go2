import cv2
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


CAMERA_FRAME_ID = 'front_camera'
CAMERA_FOV_DEG = 120.0  
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

GSTREAMER_PIPELINE = (
    "udpsrc address=230.1.1.1 port=1720 multicast-iface=eno1 ! "
    "application/x-rtp, media=video, encoding-name=H264 ! "
    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
    "video/x-raw,width=1280,height=720,format=BGR ! "
    "appsink drop=1 max-buffers=1 sync=false"
)


class GStreamerVideoNode(Node):
    def __init__(self):
        super().__init__('gstreamer_video_node')
        self.bridge = CvBridge()


        self.image_pub = self.create_publisher(Image, '/sobaka_image', 4)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/sobaka_camera_info', 4)


        self._camera_info = self._build_camera_info(CAMERA_WIDTH, CAMERA_HEIGHT)


        self.get_logger().info('Opening GStreamer pipeline...')
        self.cap = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open GStreamer pipeline!')
            raise RuntimeError('GStreamer pipeline did not open')


        self.timer = self.create_timer(1.0 / 15.0, self.on_timer)
        self.get_logger().info('GStreamer video node started, publishing to /sobaka_image')

    def _build_camera_info(self, width, height):
        fov_rad = math.radians(CAMERA_FOV_DEG)
        fx = width / (2.0 * math.tan(fov_rad / 2.0))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        info = CameraInfo()
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def on_timer(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Failed to read frame', throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = CAMERA_FRAME_ID
        self.image_pub.publish(img_msg)

        self._camera_info.header.stamp = stamp
        self._camera_info.header.frame_id = CAMERA_FRAME_ID
        self.camera_info_pub.publish(self._camera_info)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GStreamerVideoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Node error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
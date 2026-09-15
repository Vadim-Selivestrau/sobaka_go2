import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstApp, GstRtspServer, GLib


MULTICAST_ADDR = '230.1.1.1'
MULTICAST_PORT = 1720
IFACE = 'eno1'
RTSP_PORT = '8554'
RTSP_MOUNT = '/stream'


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/sobaka_image', 4)

        Gst.init(None)

        pipeline_str = (
            f'udpsrc address={MULTICAST_ADDR} port={MULTICAST_PORT} multicast-iface={IFACE} ! '
            'queue ! application/x-rtp, media=video, encoding-name=H264 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
            'video/x-raw, format=BGR ! appsink name=sink emit-signals=true'
        )
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info('ROS image publishing started: /sobaka_image')


        self.rtsp_server = GstRtspServer.RTSPServer()
        self.rtsp_server.set_service(RTSP_PORT)

        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(
            f'( udpsrc address={MULTICAST_ADDR} port={MULTICAST_PORT} multicast-iface={IFACE} ! '
            'application/x-rtp, media=video, encoding-name=H264 ! '
            'rtph264depay ! h264parse config-interval=1 ! '
            'queue max-size-buffers=1 leaky=downstream ! '
            'rtph264pay name=pay0 pt=96 )'
        )
        factory.set_shared(True)  # один пайплайн на всех клиентов, не плодим сокеты
        factory.set_latency(0)   # добавили эту строку
        mounts = self.rtsp_server.get_mount_points()
        mounts.add_factory(RTSP_MOUNT, factory)
        self.rtsp_server.attach(None)

        self.get_logger().info(
            f'RTSP stream available at rtsp://<jetson_ip>:{RTSP_PORT}{RTSP_MOUNT}'
        )


        self.glib_loop = GLib.MainLoop()
        self.glib_thread = threading.Thread(target=self.glib_loop.run, daemon=True)
        self.glib_thread.start()

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_int('width')[1]
        height = structure.get_int('height')[1]

        result, map_info = buf.map(Gst.MapFlags.READ)
        if not result:
            return Gst.FlowReturn.OK

        frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3))
        buf.unmap(map_info)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)

        return Gst.FlowReturn.OK

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        if self.glib_loop.is_running():
            self.glib_loop.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
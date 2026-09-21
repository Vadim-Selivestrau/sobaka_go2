import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import time


MODEL_PATH = '/home/jetson/sobaka/sobaka_go2/cyclonedds_ws/model/yolov8n.engine'
TARGET_CLASS_ID = 0  # person
CONF_THRESHOLD = 0.65
IMG_SIZE = 640 #yolov8n input 640, resize


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        self.model = YOLO(MODEL_PATH, task='detect')

        # прогрев
        import numpy as np
        dummy = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype='uint8')
        self.model.track(dummy, persist=True, tracker='bytetrack.yaml', verbose=False)

        self.detection_pub = self.create_publisher(
            Detection2DArray, '/tracked_targets_2d', 4
        )

        self.create_subscription(
            Image, '/sobaka_image', self.on_image, qos_profile_sensor_data
        )

        self._frame_count = 0
        self._last_fps_log = time.time()

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.track(
            frame,
            persist=True,
            tracker='bytetrack.yaml',
            classes=[TARGET_CLASS_ID],
            conf=CONF_THRESHOLD,
            imgsz=IMG_SIZE,
            verbose=False,
        )

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header  

        result = results[0]
        if result.boxes is not None and result.boxes.id is not None:
            boxes_xywh = result.boxes.xywh.cpu()
            track_ids = result.boxes.id.int().cpu().tolist()
            confs = result.boxes.conf.cpu().tolist()

            for box, track_id, conf in zip(boxes_xywh, track_ids, confs):
                x, y, w, h = box.tolist()

                det = Detection2D()
                det.header = msg.header
                det.id = str(track_id)

                det.bbox.center.position.x = x
                det.bbox.center.position.y = y
                det.bbox.size_x = w
                det.bbox.size_y = h

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = 'person'
                hyp.hypothesis.score = conf
                det.results.append(hyp)

                detections_msg.detections.append(det)

        self.detection_pub.publish(detections_msg)


        self._frame_count += 1
        now = time.time()
        if now - self._last_fps_log >= 1.0:
            self.get_logger().info(
                f'Perception FPS: {self._frame_count / (now - self._last_fps_log):.1f}, '
                f'targets tracked: {len(detections_msg.detections)}'
            )
            self._frame_count = 0
            self._last_fps_log = now


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
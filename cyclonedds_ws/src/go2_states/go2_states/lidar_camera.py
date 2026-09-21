import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, TransformStamped
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_ros import TransformException


BASE_FRAME = 'base_link'
BBOX_MARGIN = 0.15          
MAX_TIME_DIFF = 1.0        
MIN_POINTS_FOR_VALID_TARGET = 3
TARGET_LOST_TIMEOUT = 1.5  


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_info = None
        self.latest_cloud = None

        self.locked_track_id = None       
        self.locked_last_seen = None    

        self.create_subscription(CameraInfo, '/sobaka_camera_info', self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/tracked_targets_2d', self.on_detections, 4)

        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 4)

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_cloud(self, msg):
        self.latest_cloud = msg

    def on_detections(self, msg: Detection2DArray):
        if self.camera_info is None or self.latest_cloud is None or not msg.detections:
            self._maybe_report_target_lost()
            return

        det_t = rclpy.time.Time.from_msg(msg.header.stamp)
        cloud_t = rclpy.time.Time.from_msg(self.latest_cloud.header.stamp)
        dt = abs((det_t - cloud_t).nanoseconds) / 1e9
        if dt > MAX_TIME_DIFF:
            self.get_logger().warn(f'рассинхрон {dt:.2f}, пропуск кадра', throttle_duration_sec=2.0)
            return

        cloud_frame = self.latest_cloud.header.frame_id
        camera_frame = self.camera_info.header.frame_id

        try:
            tf_cam = self.tf_buffer.lookup_transform(camera_frame, cloud_frame, rclpy.time.Time())
            tf_base = self.tf_buffer.lookup_transform(BASE_FRAME, cloud_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'трансформы упали: {e}', throttle_duration_sec=2.0)
            return

        pts_struct = pc2.read_points(self.latest_cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        points_lidar = np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=-1).astype(np.float64)
        if points_lidar.shape[0] == 0:
            return

        R_cam, t_cam = self._transform_to_rt(tf_cam)
        R_base, t_base = self._transform_to_rt(tf_base)

        points_cam = (R_cam @ points_lidar.T).T + t_cam
        points_base = (R_base @ points_lidar.T).T + t_base

        valid = points_cam[:, 2] > 0.05 
        points_cam = points_cam[valid]
        points_base = points_base[valid]
        if points_cam.shape[0] == 0:
            return

        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        u = (points_cam[:, 0] * fx / points_cam[:, 2]) + cx
        v = (points_cam[:, 1] * fy / points_cam[:, 2]) + cy


        candidates = {}  # (median_point_base, n_points)
        for det in msg.detections:
            bx, by = det.bbox.center.position.x, det.bbox.center.position.y
            bw, bh = det.bbox.size_x, det.bbox.size_y
            hw, hh = bw / 2.0 * (1.0 - BBOX_MARGIN), bh / 2.0 * (1.0 - BBOX_MARGIN)

            in_box = (u >= bx - hw) & (u <= bx + hw) & (v >= by - hh) & (v <= by + hh)
            n = int(np.sum(in_box))
            if n < MIN_POINTS_FOR_VALID_TARGET:
                continue

            median_point = np.median(points_base[in_box], axis=0)
            candidates[det.id] = (median_point, n)

        if not candidates:
            self._maybe_report_target_lost()
            return

        now = self.get_clock().now()

        if self.locked_track_id is not None:
            if self.locked_track_id in candidates:
                chosen_id = self.locked_track_id
            else:
                elapsed = (now - self.locked_last_seen).nanoseconds / 1e9
                if elapsed > TARGET_LOST_TIMEOUT:
                    chosen_id = min(candidates, key=lambda k: np.linalg.norm(candidates[k][0][:2]))
                    self.get_logger().info(f'Цель {self.locked_track_id} потеряна, переключаюсь на {chosen_id}')
                else:
                    return
        else:
            chosen_id = min(candidates, key=lambda k: np.linalg.norm(candidates[k][0][:2]))

        self.locked_track_id = chosen_id
        self.locked_last_seen = now

        

        median_point, n_points = candidates[chosen_id]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = BASE_FRAME
        pose_msg.pose.position.x = float(median_point[0])
        pose_msg.pose.position.y = float(median_point[1])
        pose_msg.pose.position.z = float(median_point[2])
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

    def _maybe_report_target_lost(self):
        if self.locked_track_id is None or self.locked_last_seen is None:
            return
        elapsed = (self.get_clock().now() - self.locked_last_seen).nanoseconds / 1e9
        if elapsed > TARGET_LOST_TIMEOUT:
            self.get_logger().warn(f'Цель {self.locked_track_id} потеряна более {TARGET_LOST_TIMEOUT}s назад, сброс')
            self.locked_track_id = None
            self.locked_last_seen = None

    @staticmethod
    def _transform_to_rt(tf_stamped: TransformStamped):
        q = tf_stamped.transform.rotation
        t = tf_stamped.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(y*y+z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),   1 - 2*(x*x+z*z),   2*(y*z - x*w)],
            [2*(x*z - y*w),   2*(y*z + x*w),     1 - 2*(x*x+y*y)],
        ])
        return R, np.array([t.x, t.y, t.z])


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
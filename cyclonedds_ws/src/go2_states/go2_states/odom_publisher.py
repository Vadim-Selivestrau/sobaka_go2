#!/usr/bin/env python3


import json
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from unitree_go.msg import SportModeState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))  # x, y, z, w


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('go2_odom_publisher')

        # --- параметры ---
        self.odom_frame_ = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame_ = self.declare_parameter('base_frame', 'base_link').value
        self.sport_topic_ = self.declare_parameter(
            'sport_mode_state_topic', '/lf/sportmodestate'
        ).value
        self.vel_threshold_ = self.declare_parameter(
            'vel_threshold', 0.03
        ).value
        self.yaw_rate_threshold_ = self.declare_parameter(
            'yaw_rate_threshold', 0.03
        ).value
        self.state_file_ = self.declare_parameter(
            'state_file', '/tmp/go2_odom_state.json'
        ).value
        self.state_save_period_ = self.declare_parameter(
            'state_save_period', 2.0  # сек, throttle периодического сохранения
        ).value

        self.offset_x_ = 0.0
        self.offset_y_ = 0.0
        self.last_good_x_ = 0.0
        self.last_good_y_ = 0.0

        self.yaw_offset_ = 0.0
        self.last_good_yaw_ = 0.0

        # Восстановление состояния с прошлого запуска (если было)
        self._load_state()

        self.odom_pub_ = self.create_publisher(
            Odometry,
            '/go2/odom_from_sport',
            rclpy.qos.qos_profile_sensor_data,
        )
        self.tf_broadcaster_ = TransformBroadcaster(self)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sport_sub_ = self.create_subscription(
            SportModeState,
            self.sport_topic_,
            self.sport_mode_callback,
            qos,
        )

        # периодическое сохранение состояния на диск (throttled),
        # чтобы штатный рестарт ноды не терял привязку к карте
        self._state_timer_ = self.create_timer(
            self.state_save_period_, self._save_state
        )

        self.get_logger().info(f'Go2OdomPublisher listening on {self.sport_topic_}')
        self.get_logger().info('Publishing odometry on /go2/odom_from_sport')
        self.get_logger().info(
            f'Restored last_good pose: '
            f'x={self.last_good_x_:.3f} y={self.last_good_y_:.3f} '
            f'yaw={self.last_good_yaw_:.3f}'
        )

    # ---------- персистентность состояния ----------

    def _load_state(self):
        if os.path.exists(self.state_file_):
            try:
                with open(self.state_file_, 'r') as f:
                    st = json.load(f)
                self.last_good_x_ = float(st.get('x', 0.0))
                self.last_good_y_ = float(st.get('y', 0.0))
                self.last_good_yaw_ = float(st.get('yaw', 0.0))
            except Exception as e:
                self.get_logger().warn(f'Failed to load odom state: {e}')

    def _save_state(self):
        try:
            tmp_path = self.state_file_ + '.tmp'
            with open(tmp_path, 'w') as f:
                json.dump({
                    'x': self.last_good_x_,
                    'y': self.last_good_y_,
                    'yaw': self.last_good_yaw_,
                }, f)
            os.replace(tmp_path, self.state_file_)  # атомарная замена
        except Exception as e:
            self.get_logger().warn(f'Failed to save odom state: {e}')

    # ---------- основной колбэк ----------

    def sport_mode_callback(self, msg: SportModeState):
        now = self.get_clock().now().to_msg()

        raw_x = float(msg.position[0])
        raw_y = float(msg.position[1])
        raw_z = float(msg.position[2])

        vx = float(msg.velocity[0])
        vy = float(msg.velocity[1])
        vz = float(msg.velocity[2])
        yaw_rate = float(msg.yaw_speed)

        raw_yaw = quat_to_yaw(
            float(msg.imu_state.quaternion[1]),  # x
            float(msg.imu_state.quaternion[2]),  # y
            float(msg.imu_state.quaternion[3]),  # z
            float(msg.imu_state.quaternion[0]),  # w
        )

        is_stopped = (
            abs(vx) < self.vel_threshold_
            and abs(vy) < self.vel_threshold_
            and abs(yaw_rate) < self.yaw_rate_threshold_
        )

        if is_stopped:
            self.offset_x_ = raw_x - self.last_good_x_
            self.offset_y_ = raw_y - self.last_good_y_
        else:
            self.last_good_x_ = raw_x - self.offset_x_
            self.last_good_y_ = raw_y - self.offset_y_

        corrected_x = raw_x - self.offset_x_
        corrected_y = raw_y - self.offset_y_

        if is_stopped:
            self.yaw_offset_ = raw_yaw - self.last_good_yaw_
        else:
            self.last_good_yaw_ = raw_yaw - self.yaw_offset_

        corrected_yaw = raw_yaw - self.yaw_offset_
        qx, qy, qz, qw = yaw_to_quat(corrected_yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_
        odom.child_frame_id = self.base_frame_

        odom.pose.pose.position.x = corrected_x
        odom.pose.pose.position.y = corrected_y
        odom.pose.pose.position.z = raw_z

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = yaw_rate

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = -1.0
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = -1.0

        self.odom_pub_.publish(odom)

        # ВАЖНО: TF теперь публикует corrected_x/corrected_y (было: raw),
        # синхронно с corrected orientation. Раньше здесь был рассинхрон:
        # transform.translation брался "сырым", а rotation - скорректированным.
        
        
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame_
        t.child_frame_id = self.base_frame_
        t.transform.translation.x = corrected_x
        t.transform.translation.y = corrected_y
        t.transform.translation.z = raw_z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster_.sendTransform(t)       



        # t.transform.translation.x = float(msg.position[0]) #corrected_x
        # t.transform.translation.y = float(msg.position[1]) #corrected_y
        # t.transform.translation.z = float(msg.position[2]) #raw_z


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save_state()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
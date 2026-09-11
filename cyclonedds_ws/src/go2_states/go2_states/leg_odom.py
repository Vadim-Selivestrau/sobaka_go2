#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from unitree_go.msg import LowState, SportModeState


class LegOdom(Node):
    def __init__(self):
        super().__init__('leg_odom')
        self.tf_broadcaster = TransformBroadcaster(self)


        self.odom_leg_pub = self.create_publisher(Odometry, '/odom/leg', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0
        self.last_time = None


        self.sport_seen_at = -1.0
        self.has_sport_source = False


        self.velocity_gate = 0.02
        self.yaw_rate_gate = 0.01      
        self.foot_force_gate = 5.0     
        
        
        
        self.create_subscription(LowState, '/lf/lowstate', self.lowstate_cb, 10)
        self.create_subscription(SportModeState, '/lf/sportmodestate', self.sport_cb, 10)
        self.get_logger().info('leg_odom: стабильная одометрия без шумного акселерометра')
        self._publish(self.get_clock().now())


    def sport_cb(self, msg):
        now = self.get_clock().now()
        self.sport_seen_at = now.nanoseconds / 1e9
        self.has_sport_source = True


        self.vx = float(msg.velocity[0])
        self.vy = float(msg.velocity[1])
        self.yaw_rate = float(msg.yaw_speed)


        w, x, y, z = msg.imu_state.quaternion
        self.yaw = self._quat_to_yaw(w, x, y, z)

        self._last_forces = msg.foot_force
        self._apply_zupt()
        self._integrate(now)
        self._publish(now)

    def lowstate_cb(self, msg):
        now = self.get_clock().now()


        w, x, y, z = msg.imu_state.quaternion
        self.yaw = self._quat_to_yaw(w, x, y, z)


        if self._sport_active(now):
            return
        self.vx = 0.0
        self.vy = 0.0
        gyro_z = float(msg.imu_state.gyroscope[2])
        self.yaw_rate = gyro_z if abs(gyro_z) > self.yaw_rate_gate else 0.0

        self._integrate(now)
        self._publish(now)


    def _sport_active(self, now):
        if not self.has_sport_source:
            return False
        age = now.nanoseconds / 1e9 - self.sport_seen_at
        return 0.0 <= age < 1.0

    def _apply_zupt(self):
        forces = np.array(self._last_forces, dtype=float) if hasattr(self, '_last_forces') else np.array([0.0, 0.0, 0.0, 0.0])
        if forces.size and np.max(forces) < self.foot_force_gate:
            self.vx = 0.0
            self.vy = 0.0
            self.yaw_rate = 0.0
        elif abs(self.vx) < self.velocity_gate and abs(self.vy) < self.velocity_gate:
            self.vx = 0.0
            self.vy = 0.0
            if abs(self.yaw_rate) < self.velocity_gate:
                self.yaw_rate = 0.0

    def _integrate(self, now):
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        self.x += (self.vx * c - self.vy * s) * dt
        self.y += (self.vx * s + self.vy * c) * dt
        if abs(self.yaw_rate) > self.yaw_rate_gate:
            self.yaw += self.yaw_rate * dt

    @staticmethod
    def _quat_to_yaw(w, x, y, z):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


    def _publish(self, now):
        stamp = now.to_msg()
        self._publish_odom(self.odom_leg_pub, stamp)
        self._publish_odom(self.odom_pub, stamp)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation = Vector3(x=self.x, y=self.y, z=0.0)
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def _publish_odom(self, pub, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        o.twist.twist.linear.x = self.vx
        o.twist.twist.linear.y = self.vy
        o.twist.twist.angular.z = self.yaw_rate
        pub.publish(o)


def main():
    rclpy.init()
    try:
        rclpy.spin(LegOdom())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

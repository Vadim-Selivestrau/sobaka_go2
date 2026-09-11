#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class RobotOdomTfBridge(Node):
    def __init__(self):
        super().__init__('robot_odom_tf_bridge')

        self.odom_topic_ = self.declare_parameter(
            'odom_topic', '/utlidar/robot_odom'
        ).value
        self.odom_frame_ = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame_ = self.declare_parameter('base_frame', 'base_link').value

        self.tf_broadcaster_ = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic_,
            self.odom_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Транслирую TF {self.odom_frame_} -> {self.base_frame_} '
            f'из {self.odom_topic_}'
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame_
        t.child_frame_id = self.base_frame_
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RobotOdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
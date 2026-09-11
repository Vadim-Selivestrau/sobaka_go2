#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CloudFrameFixer(Node):
    def __init__(self):
        super().__init__('cloud_frame_fixer')
        self.target_frame_ = self.declare_parameter(
            'target_frame', 'utlidar_lidar'
        ).value

        self.sub_ = self.create_subscription(
            PointCloud2, '/utlidar/cloud_deskewed', self.cb, 10
        )
        self.pub_ = self.create_publisher(
            PointCloud2, '/utlidar/cloud_deskewed_fixed', 10
        )

    def cb(self, msg: PointCloud2):
        msg.header.frame_id = self.target_frame_
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
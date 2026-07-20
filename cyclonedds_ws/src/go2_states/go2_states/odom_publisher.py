import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/utlidar/robot_pose', self.pose_callback, 10
        )

    def pose_callback(self, msg):

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = msg.pose
        self.odom_pub.publish(odom)


        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation = Vector3(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
        )
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
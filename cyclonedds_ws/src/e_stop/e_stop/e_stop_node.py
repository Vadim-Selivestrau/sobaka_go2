import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class DualSenseEStop(Node):
    def __init__(self):
        super().__init__('dualsense_estop')
        
        # State tracking
        self.estop_active = False
        
        # Subscriptions
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        
        self.get_logger().info("DualSense E-Stop Node Started. Press SQUARE to toggle E-STOP.")

    def joy_callback(self, msg):
        # DualSense Square button is index 3 on Linux joy_node
        square_pressed = msg.buttons[0] == 1
        
        # If pressed, activate E-STOP (Latch system)
        if square_pressed and not self.estop_active:
            self.estop_active = True
            self.get_logger().warn("EMERGENCY STOP ACTIVATED BY DUALSENSE!")
            self.publish_estop_status()

    def vel_callback(self, msg):
        out_msg = Twist()
        
        # Zero out velocity if E-Stop is active
        if self.estop_active:
            out_msg.linear.x = 0.0
            out_msg.angular.z = 0.0
        else:
            out_msg = msg
            
        self.vel_pub.publish(out_msg)

    def publish_estop_status(self):
        status = Bool()
        status.data = self.estop_active
        self.estop_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = DualSenseEStop()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

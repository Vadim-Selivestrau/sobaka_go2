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
        self.check = False
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        
        self.get_logger().info("DualSense E-Stop Node Started. Press SQUARE to toggle E-STOP.")

    def joy_callback(self, msg):

        square_pressed = msg.buttons[0] == 1
        
        if square_pressed:
            self.check = True

        if self.check and not square_pressed and not self.estop_active:
            self.estop_active = True
            self.get_logger().warn("EMERGENCY STOP ACTIVATED BY DUALSENSE!")
            self.publish_estop_status()
            self.check = False
        
        if self.check and not square_pressed and self.estop_active:
            self.estop_active = False
            self.get_logger().warn("change!")
            self.publish_estop_status()
            self.check = False
           

        # if not square_pressed:
        #     pressed = False
        # if self.estop_active and square_pressed and self.check:
        #     self.check = False
        #     self.estop_active = False
        #     self.get_logger().warn("estop not blocked")
        #     self.publish_estop_status()


    def vel_callback(self, msg):
        out_msg = Twist()
        

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

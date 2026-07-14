import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
from std_srvs.srv import Empty
import json

class JoyHandler(Node):
    def __init__(self):
        super().__init__('joy_handler')
        self.joy_sub = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10
        )

        self.sport_mode_pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )



    def joy_callback(self, msg: Joy):
        self.last_cmd_time = self.get_clock().now()
        self.stopped = False




        R2 = msg.axes[5]   # 1..-1
        L2 = msg.axes[2]   # 1..-1
        L1 = msg.buttons[4]
        R1 = msg.buttons[5]
        R2_button = msg.buttons[7]
        L2_button = msg.buttons[6]
        left_stick_x = msg.axes[0]
        left_stick_y = msg.axes[1]

        right_stick_x = msg.axes[3]
        right_stick_y = msg.axes[4]

        cross  = msg.buttons[0]
        square  = msg.buttons[3]
        triangle  = msg.buttons[2]
        circle  = msg.buttons[1]
        if msg.axes[7] < 0:
            arrow_down = msg.axes[7]
        elif msg.axes[7] > 0:
            arrow_up = msg.axes[7]

        if msg.axes[6] < 0:
            arrow_right  = msg.axes[6]
        elif msg.axes[6] > 0:
            arrow_left = msg.axes[6]
        

        R3 = msg.buttons[12]
        L3 = msg.buttons[11]
        option_button = msg.buttons[9]
        share_button = msg.buttons[8]
        PS_button = msg.buttons[10]


        vx, vy, yaw = left_stick_x, left_stick_y, right_stick_x
        cur_twist = Twist()
        cur_twist.linear.x = vy
        cur_twist.linear.y = vx
        cur_twist.angular.z = yaw
        
        
        params = {}
        sport_mode = Request()
        
        
        if msg.axes[6] > 0.0:
            sport_mode.header.identity.api_id = 1004 # нормал моде
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)
        elif msg.axes[6] < 0.0:
            sport_mode.header.identity.api_id = 1002 # баланс моде
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)


        if msg.axes[7] > 0.0:
            sport_mode.header.identity.api_id = 1004 # встать
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)            
        elif msg.axes[7] < 0.0:
            sport_mode.header.identity.api_id = 1005 # сесть
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)

        if square > 0.0:
            sport_mode.header.identity.api_id = 1009 # сесть
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)  

        if triangle > 0.0:
            sport_mode.header.identity.api_id = 1017 # Потянуться
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)  

        if circle > 0.0:
            sport_mode.header.identity.api_id = 1016 # hello
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)  

        if cross > 0.0:
            sport_mode.header.identity.api_id = 1033 # виляние
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)  



        # euler eagl
        # roll: Range [-0.75~0.75] (rad); pitch: Range [-0.75~0.75] (rad); yaw: Range [-0.6~0.6] (rad).

        if R1 > 0.0:
            sport_mode.header.identity.api_id = 1007
            params["roll"] = left_stick_x
            params["pitch"] = left_stick_y
            params["yaw"] = right_stick_y
            sport_mode.parameter = json.dumps(params, separators=(',', ':'))

            self.sport_mode_pub.publish(sport_mode)  


        print(vx, vy, yaw)

        if abs(vx) > 0.1 or abs(vy) > 0.1 or abs(yaw) > 0.1:
            self.cmd_pub.publish(cur_twist)

    # "Damp": 1001,
    # "BalanceStand": 1002,
    # "StopMove": 1003,
    # "StandUp": 1004,
    # "StandDown": 1005,
    # "RecoveryStand": 1006,
    # "Euler": 1007,
    # "Move": 1008,
    # "Sit": 1009,
    # "RiseSit": 1010,
    # "SwitchGait": 1011,
    # "Trigger": 1012,
    # "BodyHeight": 1013,
    # "FootRaiseHeight": 1014,
    # "SpeedLevel": 1015,
    # "Hello": 1016,
    # "Stretch": 1017,
    # "TrajectoryFollow": 1018,
    # "ContinuousGait": 1019,
    # "Content": 1020,
    # "Wallow": 1021,
    # "Dance1": 1022,
    # "Dance2": 1023,
    # "GetBodyHeight": 1024,
    # "GetFootRaiseHeight": 1025,
    # "GetSpeedLevel": 1026,
    # "SwitchJoystick": 1027,
    # "Pose": 1028,
    # "Scrape": 1029,
    # "FrontFlip": 1030,
    # "FrontJump": 1031,
    # "FrontPounce": 1032,
    # "WiggleHips": 1033,
    # "GetState": 1034,
    # "EconomicGait": 1035,
    # "FingerHeart": 1036,
    # "StandOut": 1039,
    # "FreeWalk": 1045,
    # "Standup": 1050,
    # "CrossWalk": 1051,
    # "Bound": 1304,
    # "MoonWalk": 1305,
    # "OnesidedStep": 1303,
    # "CrossStep": 1302,
    # "Handstand": 1301,


def main():
    rclpy.init()
    node = JoyHandler()
    rclpy.spin(node)



if __name__ == '__main__':
    main()
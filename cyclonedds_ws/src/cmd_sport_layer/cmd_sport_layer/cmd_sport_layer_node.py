import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
import json


class CmdSportLayer(Node):
    def __init__(self):
        super().__init__('cmd_sport_layer')
        
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_callback,
            10
        )


        self.sport_mode_pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )
        
        self.timeout_sec = 0.3
        self.last_cmd_time = self.get_clock().now() 
        self.stopped = False

        self.timer = self.create_timer(0.1, self.timer_callback) 

    def cmd_callback(self, msg: Twist):

        self.last_cmd_time = self.get_clock().now()
        self.stopped = False

        sport_mode = Request()


        max_lin = 1.5 #0.9   
        max_ang = 2.0

        vx = max(-max_lin, min(max_lin, msg.linear.x))
        vy = max(-max_lin, min(max_lin, msg.linear.y))
        yaw = max(-max_ang, min(max_ang, msg.angular.z))
    
        params = {}
        params["x"] = vx
        params["y"] = vy
        params["z"] = yaw


        
        
    

        sport_mode.header.identity.api_id = 1008
        sport_mode.parameter = json.dumps(params, separators=(',', ':'))

        self.sport_mode_pub.publish(sport_mode)
    
    def timer_callback(self):
        if self.stopped:
            return

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9  

        if dt > self.timeout_sec:
            stop_cmd = Request()
            params = {}
            params["x"] = 0.0
            params["y"] = 0.0
            params["z"] = 0.0

            stop_cmd.header.identity.api_id = 1008
            text = json.dumps(params, separators=(',', ':'))
            self.get_logger().info(f'{text}')

            stop_cmd.parameter = json.dumps(params, separators=(',', ':'))
            


            self.sport_mode_pub.publish(stop_cmd)
            
            self.get_logger().warn(f'No cmd_vel for {dt:.2f}s, sending stop!')
            self.sport_mode_pub.publish(stop_cmd)
            stop_cmd.header.identity.api_id = 1002 # баланс моде
            stop_cmd.parameter = json.dumps(params, separators=(',', ':'))
            self.sport_mode_pub.publish(stop_cmd)
            self.get_logger().info(f'баланс моде, если ничего не отвечает? ЗАЩИТИТ ОТ ПАДЕНИЯ')
            self.stopped = True 

def main():
    rclpy.init()
    node = CmdSportLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

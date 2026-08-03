import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState
from sensor_msgs.msg import JointState

THRESHOLD = 12

class Go2JointStatePublisher(Node):

    def __init__(self):
        super().__init__('go2_joint_state_publisher')

        self.counter = 0
        self.sub = self.create_subscription(LowState, '/lowstate', self.callback, 1)
        self.pub = self.create_publisher(JointState, '/joint_states', 1)

        self.joint_names = [
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        ]
#theroboverse
    def callback(self, msg):
        if len(msg.motor_state) < 12:
            return
        print(self.counter)
        self.counter += 1
        if self.counter == THRESHOLD:
            #cчётчик на каждое 12е сообщение
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names

            js.position = [msg.motor_state[i].q for i in range(12)]
            self.counter = 0
            self.pub.publish(js)

def main():
    rclpy.init()
    rclpy.spin(Go2JointStatePublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

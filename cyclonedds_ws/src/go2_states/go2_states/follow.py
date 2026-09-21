import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


# --- Параметры следования ---
DESIRED_DISTANCE = 1.5       # м, целевая дистанция до человека
KP_LINEAR = 0.5
KP_ANGULAR = 1.2
MAX_LINEAR_VEL = 0.6         # м/с
MAX_ANGULAR_VEL = 1.0        # рад/с
DISTANCE_DEADZONE = 0.15     # м, не дёргаемся в пределах этой зоны вокруг DESIRED_DISTANCE

# --- Safety ---
TARGET_TIMEOUT = 1.0         # сек, если /target_pose не приходит дольше — стоп
MIN_SAFE_DISTANCE = 0.5      # м, минимальная дистанция до цели (аварийный стоп/отход)
OBSTACLE_CHECK_WIDTH = 0.5   # м, полуширина "коридора" перед роботом, где проверяем препятствия
OBSTACLE_STOP_DISTANCE = 0.2 # м, дистанция до препятствия, при которой останавливаемся
OBSTACLE_SLOW_DISTANCE = 1.0 # м, дистанция, с которой начинаем снижать скорость


class FollowController(Node):
    def __init__(self):
        super().__init__('follow_controller')

        self.last_target = None
        self.last_target_time = None
        self.latest_cloud = None

        self.create_subscription(PoseStamped, '/target_pose', self.on_target, 4)
        self.create_subscription(PointCloud2, '/utlidar/cloud', self.on_cloud, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 4)

        # Отдельный таймер — контроллер работает на своей частоте, не завязан на приходе target_pose,
        # чтобы гарантированно останавливать робота даже если сообщения перестали приходить вовсе
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self._state = 'IDLE'
        self._last_logged_state = None

    def on_target(self, msg: PoseStamped):
        self.last_target = msg
        self.last_target_time = self.get_clock().now()

    def on_cloud(self, msg: PointCloud2):
        self.latest_cloud = msg

    def _get_forward_obstacle_distance(self):
        """Минимальная дистанция до препятствия в узком коридоре прямо перед роботом (base_link, +X вперёд)."""
        if self.latest_cloud is None:
            return None

        pts = pc2.read_points(self.latest_cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return None

        x = pts['x']
        y = pts['y']
        z = pts['z']

        # Точки впереди робота, в пределах коридора по Y, и в разумном диапазоне по высоте
        # (чтобы не считать препятствием пол/потолок из-за шума лидара)
        in_corridor = (x > 0.1) & (np.abs(y) < OBSTACLE_CHECK_WIDTH) & (z > -0.3) & (z < 1.0)
        if not np.any(in_corridor):
            return None

        return float(np.min(x[in_corridor]))

    def control_loop(self):
        now = self.get_clock().now()
        cmd = Twist()

        # --- Проверка свежести цели ---
        target_fresh = (
            self.last_target is not None
            and (now - self.last_target_time).nanoseconds / 1e9 < TARGET_TIMEOUT
        )

        if not target_fresh:
            self._set_state('TARGET_LOST')
            self.cmd_pub.publish(cmd)  # нулевой Twist — стоим на месте
            return

        target = self.last_target.pose.position
        distance = math.sqrt(target.x ** 2 + target.y ** 2)
        bearing = math.atan2(target.y, target.x)

        # --- Аварийная зона: слишком близко к цели ---
        if distance < MIN_SAFE_DISTANCE:
            self._set_state('TOO_CLOSE')
            cmd.linear.x = 0.0
            cmd.angular.z = KP_ANGULAR * bearing  # разрешаем довернуться, но не ехать вперёд
            cmd.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, cmd.angular.z))
            self.cmd_pub.publish(cmd)
            return

        # --- Основной P-регулятор ---
        distance_error = distance - DESIRED_DISTANCE
        if abs(distance_error) < DISTANCE_DEADZONE:
            distance_error = 0.0

        linear_x = KP_LINEAR * distance_error
        angular_z = KP_ANGULAR * bearing

        linear_x = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, linear_x))
        angular_z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_z))

        # --- Safety override по лидару: препятствие прямо по курсу движения вперёд ---
        # if linear_x > 0.0:
        #     obstacle_dist = self._get_forward_obstacle_distance()
        #     if obstacle_dist is not None:
        #         if obstacle_dist < OBSTACLE_STOP_DISTANCE:
        #             linear_x = 0.0
        #             self._set_state('OBSTACLE_BLOCKED')
        #         elif obstacle_dist < OBSTACLE_SLOW_DISTANCE:
        #             # линейное снижение скорости по мере приближения к препятствию
        #             scale = (obstacle_dist - OBSTACLE_STOP_DISTANCE) / (OBSTACLE_SLOW_DISTANCE - OBSTACLE_STOP_DISTANCE)
        #             linear_x *= scale
        #             self._set_state('OBSTACLE_SLOWING')
        #         else:
        #             self._set_state('TRACKING')
        #     else:
        #         self._set_state('TRACKING')
        # else:
        self._set_state('TRACKING')

        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        if cmd.linear.x != 0.0:
            self.cmd_pub.publish(cmd)

    def _set_state(self, state):
        self._state = state
        if state != self._last_logged_state:
            self.get_logger().info(f'State: {state}')
            self._last_logged_state = state


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
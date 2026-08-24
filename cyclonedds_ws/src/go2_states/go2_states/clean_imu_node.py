#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

# Импортируем стандартный тип IMU и кастомный тип Unitree
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState

class CleanImuNode(Node):
    def __init__(self):
        super().__init__('clean_imu_node')
        
        # Подписка на низкоуровневое состояние робота
        self.subscription = self.create_subscription(
            LowState,
            '/lf/lowstate',
            self.lowstate_callback,
            10
        )
        
        # Публикация очищенных данных IMU в стандартном формате ROS 2
        self.publisher = self.create_publisher(
            Imu,
            '/imu/data_clean',
            10
        )
        
        self.get_logger().info('Узел очистки IMU успешно запущен и слушает /lowstate...')

    def lowstate_callback(self, msg):
        # 1. Извлекаем данные из сообщения Unitree
        # Структура кватерниона в unitree_go: [w, x, y, z]
        w, x, y, z = msg.imu_state.quaternion
        acc_raw = np.array(msg.imu_state.accelerometer)
        gyro = msg.imu_state.gyroscope

        # Защита от пустых или некорректных данных при старте
        if np.all(acc_raw == 0.0) or (w == 0.0 and x == 0.0 and y == 0.0 and z == 0.0):
            return

        # 2. Строим матрицу поворота (Rotation Matrix) из кватерниона
        # Она переводит векторы из глобальной системы координат в локальную робота
        R = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x**2 + z**2),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

        # 3. Считаем модуль вектора гравитации конкретно вашего чипа (около 9.52 м/с²)
        # Это исключает остаточный шум из-за заводской погрешности масштаба
        g_magnitude = np.linalg.norm(acc_raw)
        g_global = np.array([0.0, 0.0, g_magnitude])

        # 4. Проецируем глобальную гравитацию на текущий наклон робота
        # Используем транспонированную матрицу R.T (перевод Global -> Local)
        g_local = R.T @ g_global

        # 5. Вычитаем гравитацию, получая ЧИСТОЕ ЛИНЕЙНОЕ ускорение
        acc_linear = acc_raw - g_local

        # 6. Формируем стандартное ROS 2 сообщение
        clean_imu_msg = Imu()
        clean_imu_msg.header.stamp = self.get_clock().now().to_msg()
        clean_imu_msg.header.frame_id = 'base_link' # Корпус робота

        # Переносим отфильтрованную ориентацию (она в порядке)
        clean_imu_msg.orientation.w = float(w)
        clean_imu_msg.orientation.x = float(x)
        clean_imu_msg.orientation.y = float(y)
        clean_imu_msg.orientation.z = float(z)

        # Переносим угловые скорости с гироскопа
        clean_imu_msg.angular_velocity.x = float(gyro[0])
        clean_imu_msg.angular_velocity.y = float(gyro[1])
        clean_imu_msg.angular_velocity.z = float(gyro[2])

        # Записываем ОЧИЩЕННОЕ линейное ускорение
        clean_imu_msg.linear_acceleration.x = float(acc_linear[0])
        clean_imu_msg.linear_acceleration.y = float(acc_linear[1])
        clean_imu_msg.linear_acceleration.z = float(acc_linear[2])

        # Зануляем матрицы ковариации (или можете заполнить их шумом датчика при необходимости)
        clean_imu_msg.orientation_covariance = [0.0] * 9
        clean_imu_msg.angular_velocity_covariance = [0.0] * 9
        clean_imu_msg.linear_acceleration_covariance = [0.0] * 9

        # Публикуем очищенный топик
        self.publisher.publish(clean_imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CleanImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

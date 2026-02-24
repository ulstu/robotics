import rclpy
import traceback
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ulstu_turtlebot.pid_controller import PIDController


class WallFollowerNode(Node):
    """
    Нода для следования вдоль стенки с использованием PID регулятора
    """
    
    def __init__(self):
        try:
            super().__init__('wall_follower_node')
            self.get_logger().info('Wall Follower Node initialization')
            
            # Параметры следования вдоль стенки
            self.declare_parameter('target_distance', 0.5)  # Желаемое расстояние до стенки (м)
            self.declare_parameter('linear_speed', 0.3)     # Линейная скорость (м/с)
            self.declare_parameter('follow_side', 'right')  # Сторона следования: 'left' или 'right'
            self.declare_parameter('max_linear_speed', 0.5)  # Максимальная линейная скорость (м/с)
            self.declare_parameter('max_angular_speed', 1.5) # Максимальная угловая скорость (рад/с)
            self.declare_parameter('min_front_clearance', 0.35)  # Минимальный зазор спереди (м)
            self.declare_parameter('min_rear_clearance', 0.30)   # Минимальный зазор сзади (м)
            self.declare_parameter('search_back_speed', 0.08)    # Скорость движения назад при поиске (м/с)
            self.declare_parameter('search_rotate_speed', 0.55)  # Скорость вращения при поиске (рад/с)
            
            # PID коэффициенты
            self.declare_parameter('kp', 1.0)
            self.declare_parameter('ki', 0.01)
            self.declare_parameter('kd', 0.3)
            
            # Получение параметров
            self.target_distance = self.get_parameter('target_distance').value
            self.linear_speed = self.get_parameter('linear_speed').value
            self.follow_side = self.get_parameter('follow_side').value
            self.max_linear_speed = self.get_parameter('max_linear_speed').value
            self.max_angular_speed = self.get_parameter('max_angular_speed').value
            self.min_front_clearance = self.get_parameter('min_front_clearance').value
            self.min_rear_clearance = self.get_parameter('min_rear_clearance').value
            self.search_back_speed = self.get_parameter('search_back_speed').value
            self.search_rotate_speed = self.get_parameter('search_rotate_speed').value
            
            kp = self.get_parameter('kp').value
            ki = self.get_parameter('ki').value
            kd = self.get_parameter('kd').value
            
            # Создание PID контроллера с ограничением угловой скорости и интеграла
            self.pid = PIDController(
                kp=kp, ki=ki, kd=kd, 
                output_limits=(-self.max_angular_speed, self.max_angular_speed),
                integral_limits=(-5.0, 5.0)  # Ограничение интеграла
            )
            
            # Настройка QoS
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # Подписчик на данные лидара
            self._scan_subscriber = self.create_subscription(
                LaserScan,
                '/scan',
                self._scan_callback,
                qos
            )
            
            # Издатель команд скорости
            qos_cmd = QoSProfile(depth=10)
            qos_cmd.reliability = QoSReliabilityPolicy.RELIABLE
            self._cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_cmd)
            
            # Переменные для хранения времени
            self._last_time = self.get_clock().now()
            self._scan_data = None
            self._wall_lost_cycles = 0
            
            self.get_logger().info(f'Wall Follower Node initialized successfully')
            self.get_logger().info(f'Target distance: {self.target_distance}m, Side: {self.follow_side}')
            self.get_logger().info(f'PID gains - Kp: {kp}, Ki: {ki}, Kd: {kd}')
            
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))
    
    def _scan_callback(self, msg):
        """
        Callback для обработки данных лидара
        """
        try:
            # Проверка на валидность данных лидара
            if msg is None or len(msg.ranges) == 0:
                self.get_logger().warning('Empty scan message received')
                return
            
            self._scan_data = msg

            front_distance = self._get_front_distance(msg)
            rear_distance = self._get_rear_distance(msg)
            
            # Вычисление расстояния до стенки сбоку
            wall_distance = self._get_side_distance(msg)
            
            if wall_distance is None:
                self._wall_lost_cycles += 1

                # Если не видим стену: очень аккуратно двигаемся назад только первые 2 цикла,
                # затем вращаемся на месте в сторону поиска стены.
                can_move_back = rear_distance is None or rear_distance > self.min_rear_clearance
                can_move_front = front_distance is None or front_distance > self.min_front_clearance

                if self._wall_lost_cycles <= 2 and can_move_back and can_move_front:
                    self.get_logger().warning(
                        'No wall detected, backing up slowly to reacquire wall'
                    )
                    self._publish_velocity(-abs(self.search_back_speed), 0.0)
                else:
                    turn = abs(self.search_rotate_speed)
                    # Для right-wall делаем вращение по часовой (отрицательная z), для left-wall наоборот.
                    turn = -turn if self.follow_side == 'right' else turn
                    self.get_logger().warning('No wall detected, rotating in place to find wall')
                    self._publish_velocity(0.0, turn)
                return

            self._wall_lost_cycles = 0
            
            if math.isinf(wall_distance) or math.isnan(wall_distance):
                self.get_logger().warning(f'Invalid wall distance value: {wall_distance}, stopping robot')
                self._publish_velocity(0.0, 0.0)
                return
            
            # Вычисление временного шага
            current_time = self.get_clock().now()
            dt = (current_time - self._last_time).nanoseconds / 1e9
            self._last_time = current_time
            
            if dt <= 0:
                dt = 0.1  # Защита от деления на ноль
            
            # Вычисление управляющего сигнала с помощью PID
            angular_velocity = self.pid.compute(self.target_distance, wall_distance, dt)
            
            # Инверсия для правой стороны (если стена справа, нужно поворачивать влево при увеличении расстояния)
            if self.follow_side == 'right':
                angular_velocity = -angular_velocity

            # Защита от столкновения спереди: если близко препятствие, не едем вперед.
            if front_distance is not None and front_distance < self.min_front_clearance:
                avoid_turn = abs(self.search_rotate_speed)
                avoid_turn = -avoid_turn if self.follow_side == 'right' else avoid_turn
                self.get_logger().warning(
                    f'Front obstacle too close ({front_distance:.2f}m), rotating to avoid collision'
                )
                self._publish_velocity(0.0, avoid_turn)
                return
            
            # Публикация команды скорости
            self._publish_velocity(self.linear_speed, angular_velocity)
            
            # Логирование (каждую секунду)
            if int(current_time.nanoseconds / 1e9) % 2 == 0:
                self.get_logger().info(
                    f'Wall distance: {wall_distance:.3f}m, '
                    f'Target: {self.target_distance:.3f}m, '
                    f'Angular vel: {angular_velocity:.3f} rad/s'
                )
            
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))
    
    def _get_side_distance(self, scan_msg):
        """
        Получение расстояния до стенки сбоку робота
        
        Args:
            scan_msg (LaserScan): Сообщение с данными лидара
            
        Returns:
            float: Среднее расстояние до стенки сбоку или None
        """
        try:
            if self.follow_side == 'left':
                return self._get_distance_in_sector(scan_msg, center_deg=90.0, half_width_deg=20.0)
            return self._get_distance_in_sector(scan_msg, center_deg=-90.0, half_width_deg=20.0)
            
        except Exception as err:
            self.get_logger().error(f'Error in _get_side_distance: {err}')
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))
            return None
    
    def _get_rear_distance(self, scan_msg):
        """
        Получение расстояния до препятствий сзади робота
        
        Args:
            scan_msg (LaserScan): Сообщение с данными лидара
            
        Returns:
            float: Минимальное расстояние до препятствий сзади или None
        """
        try:
            return self._get_distance_in_sector(scan_msg, center_deg=180.0, half_width_deg=20.0)
            
        except Exception as err:
            self.get_logger().error(f'Error in _get_rear_distance: {err}')
            return None

    def _get_front_distance(self, scan_msg):
        """Минимальная дистанция до препятствия спереди."""
        try:
            # Сектор спереди вокруг 0°
            return self._get_distance_in_sector(scan_msg, center_deg=0.0, half_width_deg=20.0)
        except Exception as err:
            self.get_logger().error(f'Error in _get_front_distance: {err}')
            return None

    def _get_distance_in_sector(self, scan_msg, center_deg, half_width_deg):
        """
        Робастное вычисление расстояния в угловом секторе независимо от конфигурации LaserScan.
        Возвращает 20-й перцентиль, чтобы снизить влияние выбросов.
        """
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return None

        # Углы лучей (рад)
        angles = scan_msg.angle_min + np.arange(ranges.size, dtype=np.float32) * scan_msg.angle_increment

        center = math.radians(center_deg)
        half = math.radians(half_width_deg)

        # Нормализация в [-pi, pi]
        angle_diff = (angles - center + math.pi) % (2.0 * math.pi) - math.pi
        mask_sector = np.abs(angle_diff) <= half

        sector = ranges[mask_sector]
        if sector.size == 0:
            return None

        valid = sector[
            np.isfinite(sector)
            & (sector > max(0.05, float(scan_msg.range_min)))
            & (sector < float(scan_msg.range_max))
        ]
        if valid.size == 0:
            return None

        return float(np.percentile(valid, 20))
    
    def _publish_velocity(self, linear, angular):
        """
        Публикация команды скорости с проверкой ограничений
        
        Args:
            linear (float): Линейная скорость (м/с)
            angular (float): Угловая скорость (рад/с)
        """
        try:
            # Явное ограничение скоростей
            linear = max(-self.max_linear_speed, min(linear, self.max_linear_speed))
            angular = max(-self.max_angular_speed, min(angular, self.max_angular_speed))
            
            twist = Twist()
            twist.linear.x = float(linear)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(angular)
            self._cmd_vel_publisher.publish(twist)
            
        except Exception as err:
            self.get_logger().error(f'Error publishing velocity: {err}')


def main(args=None):
    try:
        rclpy.init(args=args)
        wall_follower_node = WallFollowerNode()
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        print('Wall follower node stopped cleanly')
    except Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))
    finally:
        if rclpy.ok():
            rclpy.shutdown()

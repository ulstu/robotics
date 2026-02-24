import rclpy
import traceback
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose


class UlstuTurtlebotNode(Node):
    def __init__(self):
        try:
            super().__init__('ulstu_turtlebot_node')
            self.get_logger().info('TurtleSim Node initialization')

            # Настройка QoS
            qos = QoSProfile(depth=10)
            qos.reliability = QoSReliabilityPolicy.RELIABLE

            # Создание издателя
            self._twist_publisher = self.create_publisher(Twist, "/cmd_vel", qos)
            self._tick = 1

            # Создание Action Client для навигации
            self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.get_logger().info('Waiting for navigate_to_pose action server...')
            self._nav_to_pose_client.wait_for_server()
            self.get_logger().info('navigate_to_pose action server available!')

            # Флаг для отправки цели только один раз
            self._goal_sent = False

            # Создание таймера
            timer_period = 0.5
            self._timer = self.create_timer(timer_period, self._node_callback)
            self.get_logger().info('Timer created successfully')

            self.get_logger().info('TurtleSim Node initialized successfully')
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _node_callback(self):
        try:
            #self.get_logger().info(f'sim_node tick {self._tick}')
            
            # Отправить цель навигации один раз после запуска
            if not self._goal_sent and self._tick > 40:  # Ждем 40 тиков перед отправкой
                self.navigate_to_point(x=2.0, y=1.0, yaw=0.0)
                self._goal_sent = True
            
            self._tick += 1
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def navigate_to_point(self, x, y, yaw=0.0):
        """
        Отправить робота в точку с координатами (x, y) и ориентацией yaw
        
        Args:
            x (float): Координата X в системе координат map
            y (float): Координата Y в системе координат map
            yaw (float): Ориентация в радианах (по умолчанию 0.0)
        """
        try:
            # Создание цели навигации
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            # Установка позиции
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.position.z = 0.0
            
            # Установка ориентации (quaternion из yaw)
            # q = [0, 0, sin(yaw/2), cos(yaw/2)]
            import math
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            self.get_logger().info(f'Sending navigation goal: x={x}, y={y}, yaw={yaw}')
            
            # Отправка цели асинхронно
            send_goal_future = self._nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )
            send_goal_future.add_done_callback(self._goal_response_callback)
            
        except Exception as err:
            self.get_logger().error(f'Error sending navigation goal: {err}')
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _goal_response_callback(self, future):
        """Callback при получении ответа от action сервера"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning('Goal was rejected!')
                return
            
            self.get_logger().info('Goal accepted! Robot is navigating...')
            
            # Ожидание результата
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._get_result_callback)
            
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _get_result_callback(self, future):
        """Callback при получении результата навигации"""
        try:
            result = future.result().result
            self.get_logger().info('Navigation completed!')
            
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _feedback_callback(self, feedback_msg):
        """Callback для получения feedback во время навигации"""
        feedback = feedback_msg.feedback
        # Можно логировать прогресс, но не делаем это слишком часто
        # self.get_logger().info(f'Navigation feedback received')
        pass


def main(args=None):
    try:
        rclpy.init(args=args)
        sim_node = UlstuTurtlebotNode()
        rclpy.spin(sim_node)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))

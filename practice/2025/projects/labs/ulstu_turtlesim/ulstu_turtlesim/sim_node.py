import rclpy
import traceback
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimNode(Node):
    def __init__(self):
        try:
            super().__init__('sim_node')
            self.get_logger().info('TurtleSim Node initialization')

            # Настройка QoS
            qos = QoSProfile(depth=10)
            qos.reliability = QoSReliabilityPolicy.RELIABLE

            # Создание издателя
            self._twist_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", qos)
            self._tick = 1

            # Создание таймера
            timer_period = 0.5
            self._timer = self.create_timer(timer_period, self._node_callback)
            self.get_logger().info('Timer created successfully')

            self.get_logger().info('TurtleSim Node initialized successfully')
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _node_callback(self):
        try:
            self.get_logger().info(f'sim_node tick {self._tick}')
            move_cmd = Twist()
            move_cmd.linear.x = 1.0
            move_cmd.linear.y = 0.0
            move_cmd.angular.z = 10.0 / self._tick
            self._twist_publisher.publish(move_cmd)
            self._tick += 1
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))


def main(args=None):
    try:
        rclpy.init(args=args)
        sim_node = SimNode()
        rclpy.spin(sim_node)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))

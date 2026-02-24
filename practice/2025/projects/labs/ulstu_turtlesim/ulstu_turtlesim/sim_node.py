import rclpy
import traceback
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class SimNode(Node):
    def __init__(self):
        try:
            super().__init__('sim_node')
            self._init_pose = None
            self._is_stopped = False

            self.get_logger().info('TurtleSim Node initialization')
            # print('TurtleSim Node initialization')

            # Настройка QoS
            qos = QoSProfile(depth=10)
            qos.reliability = QoSReliabilityPolicy.RELIABLE

            # Создание издателя
            self._twist_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", qos)
            self._pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self._pose_callback, 10)
            self._tick = 1

            # Создание таймера
            timer_period = 0.5
            self._timer = self.create_timer(timer_period, self._node_callback)
            self.get_logger().info('Timer created successfully')

            self.get_logger().info('TurtleSim Node initialized successfully')
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _pose_callback(self, msg):
        #self.get_logger().info(f'pose: {msg}')
        if self._init_pose is None:
            self._init_pose = msg
        dist = math.dist([msg.x, msg.y], [self._init_pose.x, self._init_pose.y])
        self.get_logger().info(f'dist: {dist}')
        if dist < 0.005 and self._tick > 2:
            self._is_stopped = True


    def _node_callback(self):
        try:
            self.get_logger().info(f'sim_node tick {self._tick}')
            move_cmd = Twist()
            if self._is_stopped:
                move_cmd.linear.x = 0.0
                move_cmd.linear.y = 0.0
                move_cmd.angular.z = 0.0 #10.0 / self._tick
            else:
                move_cmd.linear.x = 1.0
                move_cmd.linear.y = 0.0
                move_cmd.angular.z = 0.8 #10.0 / self._tick
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

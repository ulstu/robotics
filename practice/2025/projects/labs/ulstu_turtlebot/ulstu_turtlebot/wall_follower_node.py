import math
import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from ulstu_turtlebot.pid_controller import PIDController


class WallFollowerNode(Node):
    """Простое следование вдоль стены с заездом в дверной проем."""

    def __init__(self):
        try:
            super().__init__('wall_follower_node')

            # Основные параметры
            self.declare_parameter('target_distance')
            self.declare_parameter('linear_speed')
            self.declare_parameter('follow_side')
            self.declare_parameter('max_linear_speed')
            self.declare_parameter('max_angular_speed')
            self.declare_parameter('min_front_clearance')
            self.declare_parameter('search_rotate_speed')
            self.declare_parameter('side_sector_half_width_deg')
            self.declare_parameter('front_check_half_width_deg')

            # PID
            self.declare_parameter('kp')
            self.declare_parameter('ki')
            self.declare_parameter('kd')

            self.target_distance = self.get_parameter('target_distance').value
            self.linear_speed = self.get_parameter('linear_speed').value
            self.follow_side = self.get_parameter('follow_side').value
            self.max_linear_speed = self.get_parameter('max_linear_speed').value
            self.max_angular_speed = self.get_parameter('max_angular_speed').value
            self.min_front_clearance = self.get_parameter('min_front_clearance').value
            self.search_rotate_speed = self.get_parameter('search_rotate_speed').value
            self.side_sector_half_width_deg = self.get_parameter('side_sector_half_width_deg').value
            self.front_check_half_width_deg = self.get_parameter('front_check_half_width_deg').value

            kp = self.get_parameter('kp').value
            ki = self.get_parameter('ki').value
            kd = self.get_parameter('kd').value

            self.pid = PIDController(
                kp=kp,
                ki=ki,
                kd=kd,
                output_limits=(-self.max_angular_speed, self.max_angular_speed),
                integral_limits=(-5.0, 5.0),
            )

            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self._scan_subscriber = self.create_subscription(LaserScan, '/scan', self._scan_callback, qos)

            qos_cmd = QoSProfile(depth=10)
            qos_cmd.reliability = QoSReliabilityPolicy.RELIABLE
            self._cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_cmd)

            # Топики для rqt_plot
            self._pub_side_dist = self.create_publisher(Float32, '/wall_follower/side_distance', 10)
            self._pub_front_dist = self.create_publisher(Float32, '/wall_follower/front_distance', qos_cmd)
            self._pub_error = self.create_publisher(Float32, '/wall_follower/error', 10)
            self._pub_angular = self.create_publisher(Float32, '/wall_follower/angular', 10)
            self._pub_target = self.create_publisher(Float32, '/wall_follower/target_distance', 10)

            self._last_time = self.get_clock().now()
            self._log_counter = 0
            self.get_logger().info('Wall Follower Node initialized')
            self.get_logger().info(f'Side: {self.follow_side}, target: {self.target_distance} m')

        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _scan_callback(self, msg: LaserScan):
        try:
            if msg is None or len(msg.ranges) == 0:
                return

            # --- Передний сектор: проверка препятствия ---
            front_dist = self._min_in_sector(msg, center_deg=0.0, half_deg=float(self.front_check_half_width_deg))
            if front_dist is not None and front_dist < self.min_front_clearance:
                turn = abs(self.search_rotate_speed)
                turn = turn if self.follow_side == 'right' else -turn
                self.get_logger().warn(f'[FRONT] obstacle {front_dist:.2f}m < {self.min_front_clearance}m → turning angular={turn:.2f}')
                self._publish_velocity(0.0, turn)
                return

            # --- Боковой сектор: следование вдоль стены ---
            side_center = -90.0 if self.follow_side == 'right' else 90.0
            side_dist = self._min_in_sector(msg, center_deg=side_center, half_deg=float(self.side_sector_half_width_deg))

            if side_dist is None:
                turn = abs(self.search_rotate_speed)
                turn = -turn if self.follow_side == 'right' else turn
                self.get_logger().warn(f'[SEARCH] no wall → turning angular={turn:.2f}')
                self._publish_velocity(0.0, turn)
                return

            current_time = self.get_clock().now()
            dt = (current_time - self._last_time).nanoseconds / 1e9
            self._last_time = current_time
            if dt <= 0.0:
                dt = 0.1

            angular = self.pid.compute(self.target_distance, side_dist, dt)
            if self.follow_side == 'left':
                angular = -angular

            error = self.target_distance - side_dist

            # Публикация данных для rqt_plot
            self._pub_side_dist.publish(Float32(data=float(side_dist)))
            self._pub_front_dist.publish(Float32(data=float(front_dist) if front_dist is not None else 0.0))
            self._pub_error.publish(Float32(data=float(error)))
            self._pub_angular.publish(Float32(data=float(angular)))
            self._pub_target.publish(Float32(data=float(self.target_distance)))

            self._log_counter += 1
            if self._log_counter % 10 == 0:
                front_str = f'{front_dist:.2f}' if front_dist is not None else 'None'
                self.get_logger().info(
                    f'[FOLLOW] front={front_str}m  side={side_dist:.2f}m  '
                    f'target={self.target_distance:.2f}m  err={error:.2f}  '
                    f'angular={angular:.2f}'
                )

            self._publish_velocity(self.linear_speed, angular)

        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _min_in_sector(self, msg: LaserScan, center_deg: float, half_deg: float):
        """Минимальная дистанция в секторе (center_deg ± half_deg)."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        diff = (angles - math.radians(center_deg) + math.pi) % (2 * math.pi) - math.pi
        mask = (np.abs(diff) <= math.radians(half_deg))
        valid = ranges[mask]
        valid = valid[np.isfinite(valid) & (valid > 0.05) & (valid < msg.range_max)]
        return float(np.min(valid)) if valid.size > 0 else None

    def _publish_velocity(self, linear: float, angular: float):
        try:
            linear = max(-self.max_linear_speed, min(linear, self.max_linear_speed))
            angular = max(-self.max_angular_speed, min(angular, self.max_angular_speed))

            twist = Twist()
            twist.linear.x = float(linear)
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

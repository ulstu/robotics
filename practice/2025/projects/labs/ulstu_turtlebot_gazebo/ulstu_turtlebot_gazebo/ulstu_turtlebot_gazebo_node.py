import rclpy
import time
import traceback
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

class UlstuTurtlebotNode(Node):
    def __init__(self):
        try:
            super().__init__('ulstu_turtlebot_node')
            self.get_logger().info('TurtleSim Node Gazebo initialization')

            # Настройка QoS
            qos = QoSProfile(depth=10)
            qos.reliability = QoSReliabilityPolicy.RELIABLE

            # Создание издателя
            self._twist_publisher = self.create_publisher(Twist, "/cmd_vel", qos)
            self._tick = 1

            # Создание таймера
            timer_period = 0.5
            self._timer = self.create_timer(timer_period, self._node_callback)
            self.get_logger().info('Timer created successfully')

            time.sleep(2)
            navigator = BasicNavigator()

            # Set our demo's initial pose
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = -0.116
            initial_pose.pose.position.y = 8.5
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 1.0
            navigator.setInitialPose(initial_pose)
    
            navigator.waitUntilNav2Active()

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = 17.86
            goal_pose.pose.position.y = -0.77
            goal_pose.pose.orientation.w = 1.0
            goal_pose.pose.orientation.z = 0.0

            # sanity check a valid path exists
            # path = navigator.getPath(initial_pose, goal_pose)

            navigator.goToPose(goal_pose)

            # i = 0
            # while not navigator.isTaskComplete():
            #     ################################################
            #     #
            #     # Implement some code here for your application!
            #     #
            #     ################################################

            #     # Do something with the feedback
            #     i = i + 1
            #     feedback = navigator.getFeedback()
            #     if feedback and i % 5 == 0:
            #         print(
            #             'Estimated time of arrival: '
            #             + '{0:.0f}'.format(
            #                 Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
            #                 / 1e9
            #             )
            #             + ' seconds.'
            #         )

            #         # Some navigation timeout to demo cancellation
            #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #             navigator.cancelTask()

            #         # Some navigation request change to demo preemption
            #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #             goal_pose.pose.position.x = 0.0
            #             goal_pose.pose.position.y = 0.0
            #             navigator.goToPose(goal_pose)

            # # Do something depending on the return code
            # result = navigator.getResult()
            # if result == TaskResult.SUCCEEDED:
            #     print('Goal succeeded!')
            # elif result == TaskResult.CANCELED:
            #     print('Goal was canceled!')
            # elif result == TaskResult.FAILED:
            #     print('Goal failed!')
            # else:
            #     print('Goal has an invalid return status!')
            self.get_logger().info('TurtleBot Gazebo Node initialized successfully')
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))

    def _node_callback(self):
        try:
            self.get_logger().info(f'sim_node tick {self._tick}')
            # move_cmd = Twist()
            # move_cmd.linear.x = 1.0
            # move_cmd.linear.y = 0.0
            # move_cmd.angular.z = 10.0 / self._tick
            # self._twist_publisher.publish(move_cmd)
            self._tick += 1
        except Exception as err:
            self.get_logger().error(''.join(traceback.TracebackException.from_exception(err).format()))


def main(args=None):
    try:
        rclpy.init(args=args)
        sim_node = UlstuTurtlebotNode()
        rclpy.spin(sim_node)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except Exception as err:
        print(''.join(traceback.TracebackException.from_exception(err).format()))

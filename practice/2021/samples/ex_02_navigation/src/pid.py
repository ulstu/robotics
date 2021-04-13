#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

print(sys.version)


class PidControl():
    def __init__(self):
        rospy.init_node('pid control script', anonymous=False)
        rospy.loginfo("To stop press CTRL + C")

        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.left = rospy.Publisher('/left', Float64, queue_size=10)
        self.right = rospy.Publisher('/right', Float64, queue_size=10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        r = rospy.Rate(2)
        i = 1
        while not rospy.is_shutdown():
            #rospy.loginfo(i)
            move_cmd = Twist()
            move_cmd.linear.x = 1
            move_cmd.linear.y = 0
            move_cmd.angular.z = 10 / i
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            i += 1

    def laser_callback(self, msg):
        try:
            right = msg.ranges[270]
            left = msg.ranges[90]
            self.left.publish(left)
            self.right.publish(right)
            pass
        except Exception as e:
            print(e)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleSim")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        ctrl = PidControl()
    except:
        rospy.loginfo("node terminated.")
        rospy.loginfo(sys.exc_info()[:2])

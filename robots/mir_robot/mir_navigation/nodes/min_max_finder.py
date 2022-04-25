#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry

lin_min = 0.0
lin_max = 0.0
ang_min = 0.0
ang_max = 0.0


def odom_cb(msg):
    global lin_min, lin_max, ang_min, ang_max
    if lin_min > msg.twist.twist.linear.x:
        lin_min = msg.twist.twist.linear.x
    if lin_max < msg.twist.twist.linear.x:
        lin_max = msg.twist.twist.linear.x
    if ang_min > msg.twist.twist.angular.z:
        ang_min = msg.twist.twist.angular.z
    if ang_max < msg.twist.twist.angular.z:
        ang_max = msg.twist.twist.angular.z

    rospy.loginfo('linear: [%f, %f]   angular: [%f, %f]', lin_min, lin_max, ang_min, ang_max)


def main():
    rospy.init_node('min_max_finder', anonymous=True)
    rospy.Subscriber('odom', Odometry, odom_cb)
    rospy.loginfo('min_max_finde node ready and listening. now use teleop to move your robot to the limits!')
    rospy.spin()


if __name__ == '__main__':
    main()

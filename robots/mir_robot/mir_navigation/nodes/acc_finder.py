#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LIN_MAX = 1.0
ANG_MAX = 1.5  # adjust this value to the rough maximum angular velocity

state = 'stopped'
start = rospy.Time(0)


def odom_cb(msg):
    global state

    twist = msg.twist.twist
    t = (rospy.Time.now() - start).to_sec()

    if state == 'wait_for_stop':
        if -0.05 < twist.linear.x < 0.05 and -0.1 < twist.angular.z < 0.1:
            state = 'stopped'
            rospy.loginfo('state transition --> %s', state)
        return

    if state == 'backward' and twist.linear.x < -0.9 * LIN_MAX:
        rospy.loginfo('backward from 0 to %f m/s in %f sec', twist.linear.x, t)
    elif state == 'forward' and twist.linear.x > 0.9 * LIN_MAX:
        rospy.loginfo('forward from 0 to %f m/s in %f sec', twist.linear.x, t)
    elif state == 'turning_clockwise' and twist.angular.z < -0.9 * ANG_MAX:
        rospy.loginfo('turning_clockwise from 0 to %f rad/s in %f sec', twist.angular.z, t)
    elif state == 'turning_counter_clockwise' and twist.angular.z > 0.9 * ANG_MAX:
        rospy.loginfo('turning_counter_clockwise from 0 to %f rad/s in %f sec', twist.angular.z, t)
    else:
        return

    state = 'wait_for_stop'
    rospy.loginfo('state transition --> %s', state)


def cmd_vel_cb(msg):
    global state, start

    if state != 'stopped':
        return

    if msg.linear.x <= -LIN_MAX:
        start = rospy.Time.now()
        state = 'backward'
    elif msg.linear.x >= LIN_MAX:
        start = rospy.Time.now()
        state = 'forward'
    elif msg.angular.z <= -ANG_MAX:
        start = rospy.Time.now()
        state = 'turning_clockwise'
    elif msg.angular.z >= ANG_MAX:
        start = rospy.Time.now()
        state = 'turning_counter_clockwise'
    else:
        return

    rospy.loginfo('state transition --> %s', state)


def main():
    rospy.init_node('acc_finder', anonymous=True)
    rospy.Subscriber('odom', Odometry, odom_cb)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)
    rospy.loginfo('acc_finder node ready and listening. now use teleop to move your robot to the limits!')
    rospy.spin()


if __name__ == '__main__':
    main()

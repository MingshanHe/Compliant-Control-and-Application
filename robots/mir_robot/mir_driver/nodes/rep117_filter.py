#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

pub = None


def callback(msg):
    """
    Convert laser scans to REP 117 standard:
    http://www.ros.org/reps/rep-0117.html
    """
    ranges_out = []
    for dist in msg.ranges:
        if dist < msg.range_min:
            # assume "reading too close to measure",
            # although it could also be "reading invalid" (nan)
            ranges_out.append(float("-inf"))

        elif dist > msg.range_max:
            # assume "reading of no return (outside sensor range)",
            # although it could also be "reading invalid" (nan)
            ranges_out.append(float("inf"))
        else:
            ranges_out.append(dist)

    msg.ranges = ranges_out
    pub.publish(msg)


def main():
    global pub
    rospy.init_node('rep117_filter')

    pub = rospy.Publisher('scan_filtered', LaserScan, queue_size=10)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

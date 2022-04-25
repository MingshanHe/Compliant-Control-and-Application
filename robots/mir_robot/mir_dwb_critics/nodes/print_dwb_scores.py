#!/usr/bin/env python3
import rospy

from dwb_msgs.msg import LocalPlanEvaluation


def eval_cb(msg):
    print('\n\n=========================================================\n\n')
    for heading, i in zip(['best trajectory', 'worst trajectory'], [msg.best_index, msg.worst_index]):
        print('### {}\n'.format(heading))
        print('Name                 |       Raw |   Scale | Scaled Score')
        print('---------------------|-----------|---------|-------------')
        for s in msg.twists[i].scores:
            print('{:20} | {:9.4f} | {:7.4f} | {:12.4f}'.format(s.name, s.raw_score, s.scale, s.raw_score * s.scale))
        print('---------------------------------------- total: {:9.4f}'.format(msg.twists[i].total))
        print()


def main():
    rospy.init_node('print_dwb_scores', anonymous=True)
    rospy.Subscriber('move_base_node/DWBLocalPlanner/evaluation', LocalPlanEvaluation, eval_cb)
    rospy.loginfo('print_dwb_scores ready.')
    rospy.spin()


if __name__ == '__main__':
    main()

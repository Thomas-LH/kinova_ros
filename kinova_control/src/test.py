#! /usr/bin/env python
import rospy


def get_time():
    rospy.loginfo(rospy.Time.now())

if __name__ == '__main__':
    rospy.init_node('time_test')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        get_time()
        rate.sleep()
    rospy.spin()
    
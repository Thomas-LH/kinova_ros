#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped

if __name__ == '__main__':
    rospy.init_node('test_message')
    pose = rospy.wait_for_message('test_data', PoseStamped)
    rospy.loginfo(pose.pose.position)
    rospy.spin()
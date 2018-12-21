#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_time():
    rospy.loginfo(rospy.Time.now())

if __name__ == '__main__':
    rospy.init_node('time_test')
    rate = rospy.Rate(1)
    test_pose = PoseStamped()
    test_pose.header.frame_id = 'world'
    test_pose.header.stamp = rospy.Time().now()
    test_pose.pose.position.x = 2
    test_pose.pose.position.y = -1
    test_pose.pose.position.z = 3.14
    test_pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,1))
    pub = rospy.Publisher('test_data', PoseStamped, queue_size=10)
    while not rospy.is_shutdown():
        #get_time()
        pub.publish(test_pose)
        rate.sleep()
    
#! /usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import argparse

PI = 3.1415926
joint_limits = {'upper':(2*PI, 313/180*PI, 2*PI, 330/180*PI, 2*PI, 295/180*PI, 2*PI),
                'lower':(-2*PI, 47/180*PI, -2*PI, 30/180*PI, -2*PI, 65/180*PI, -2*PI)}
trajectory_points = [[-1.5,2.9,0.0,1.3,4.2,1.4,0.0],
                     [0.0,2.9,0.0,1.3,4.2,1.4,0.0],
                     [1.5,2.9,0.0,1.3,4.2,1.4,0.0],
                     [0.0,2.9,0.0,1.3,4.2,1.4,0.0]]

def move_test(command):
    topic_name = '/j2s7s300/effort_joint_trajectory_controller/command'
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
    jointcmd = JointTrajectory()
    jointcmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    jointcmd.joint_names.extend(['j2s7s300_joint_' + str(i+1) for i in range(0,7)])
    points = []
    for point_num in range(len(trajectory_points)):
        points.append(JointTrajectoryPoint())
        points[point_num].time_from_start = rospy.Duration.from_sec((point_num+1)*5.0)
        points[point_num].positions.extend(command[point_num][:])
        points[point_num].velocities.extend([0]*7)
        points[point_num].accelerations.extend([0]*7)
        points[point_num].effort.extend([0]*7)
    jointcmd.points.extend(points)
    rate = rospy.Rate(100)
    count = 0
    while (count < 30):
        pub.publish(jointcmd)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('test_msg')
    rospy.sleep(1)
    '''
    home = [0.0,2.9,0.0,1.3,4.2,1.4,0.0]
    cmd = home
    for joint_num in range(7):
        cmd[joint_num] += -1.5
        if joint_limits['lower'][joint_num] > cmd[joint_num]:
            cmd[joint_num] = joint_limits['lower'][joint_num]
        move_test(cmd)
        rospy.sleep(10)
        rospy.loginfo('goal 1 finished')
        cmd = home
        move_test(cmd)
        rospy.sleep(10)
        rospy.loginfo('home goal finished')
        cmd[joint_num] += 1.5
        if joint_limits['upper'][joint_num] < cmd[joint_num]:
            cmd[joint_num] = joint_limits['upper'][joint_num]
        move_test(cmd)
        rospy.sleep(10)
        rospy.loginfo('goal 2 finished')
        cmd = home
        move_test(cmd)
        rospy.sleep(10)
        rospy.loginfo('home goal finished')
    '''
    move_test(trajectory_points)
    


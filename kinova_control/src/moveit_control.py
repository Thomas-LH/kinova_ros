#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

PI = 3.1415
joint_limits = {'upper':(2*PI, 313/180*PI, 2*PI, 330/180*PI, 2*PI, 295/180*PI, 2*PI),
                'lower':(-2*PI, 47/180*PI, -2*PI, 30/180*PI, -2*PI, 65/180*PI, -2*PI)}

def move_group_python_interface():
    print "===== Starting setup====="
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True) # initialize moveit_commander and rospy
    robot = moveit_commander.RobotCommander() # Instantiate a RobotCommander object
#    scene = moveit_commander.PlanningSceneInterface() # Instantiate a PlanningSceneInterface object
    group = moveit_commander.MoveGroupCommander("arm") # Instantiate a MoveGroupCommander object
    # publish trajectory for RVIZ to visualize
#    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1) 
    print "===== Waiting for RVIZ ====="
    rospy.sleep(10)
    print "===== Starting tutorial ====="

    print "===== Reference frame: %s" % group.get_planning_frame()
    print "===== Reference frame: %s" % group.get_end_effector_link()
    print "===== Robot Groups:"
    print robot.get_group_names()
    print "===== Printing robot state"
    print robot.get_current_state()

    print "===== Plan 1 ====="
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.600000
    pose_target.position.x = 0.500000
    pose_target.position.y = -0.2000000
    pose_target.position.z = 0.2000000
    group.set_goal_position_tolerance(0.0500000)
    group.set_goal_orientation_tolerance(0.100000)
    group.set_pose_target(pose_target, end_effector_link="j2s7s300_end_effector")
    plan1 = group.plan()

    print "===== Waiting while RVIZ displays plan1 ====="
    rospy.sleep(5)
    group.execute(plan1)
    
    print "===== Plan 2 ====="
    group.clear_pose_targets()
    group_values = group.get_current_joint_values()
    print "===== Joint values: ", group_values
    group_values[4] += 1.0
    group.set_joint_value_target(group_values)
    plan2 = group.plan()

    print "===== Waiting while RVIZ displays plan2 ====="
    rospy.sleep(5)
    group.execute(plan2)

    print "===== Plan 3 ====="
    waypoints = []
    waypoints.append(group.get_current_pose().pose)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.00
    wpose.position = waypoints[0].position
    wpose.position.x = wpose.position.x + 0.1
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.y -= 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += 0.30
    waypoints.append(copy.deepcopy(wpose))
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

    print "===== Waiting while RVIZ displays plan3 ====="
    rospy.sleep(5)
    group.execute(plan3)

    moveit_commander.roscpp_shutdown()
    print "===== STOPPING ====="

if __name__ == '__main__':
    try:
        move_group_python_interface()
    except rospy.ROSInterruptException:
        pass
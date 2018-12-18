#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import kinova_msgs.msg
from std_msgs.msg import String

PI = 3.1415
joint_limits = {'upper':(2*PI, 313/180*PI, 2*PI, 330/180*PI, 2*PI, 295/180*PI, 2*PI),
                'lower':(-2*PI, 47/180*PI, -2*PI, 30/180*PI, -2*PI, 65/180*PI, -2*PI)}

def move_group_python_interface():
    print "===== Starting setup====="
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True) # initialize moveit_commander and rospy
    robot = moveit_commander.RobotCommander() # Instantiate a RobotCommander object
    scene = moveit_commander.PlanningSceneInterface('world') # Instantiate a PlanningSceneInterface object
    group = moveit_commander.MoveGroupCommander("arm") # Instantiate a MoveGroupCommander object
    gripper = moveit_commander.MoveGroupCommander("gripper")
    # publish trajectory for RVIZ to visualize
#    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1) 
    print "===== Waiting for RVIZ ====="
    rospy.sleep(3)
    print "===== Starting test ====="
#    scene_pub = rospy.Publisher('planning_scene', moveit_msgs.msg.PlanningScene)
    ground_id = 'ground'
    rospy.sleep(1)
    ground_size = [5, 5, 0.01]
    ground_pos = geometry_msgs.msg.PoseStamped()
    ground_pos.header.frame_id = 'world'
    ground_pos.pose.position.x = 0.0
    ground_pos.pose.position.y = 0.0
    ground_pos.pose.position.z = -ground_size[2]/2.0
    ground_pos.pose.orientation.w = 1.0
    scene.add_box(ground_id, ground_pos, ground_size)

    print "===== Reference frame: %s" % group.get_planning_frame()
    print "===== Reference frame: %s" % group.get_end_effector_link()
    print "===== Robot Groups:"
    print robot.get_group_names()
    print "===== Printing robot state"
    print robot.get_current_state()

    print "===== Plan 1 ====="
    pose_target = geometry_msgs.msg.Pose()
    pose = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped)
    print pose
    pose_vals = str2vals(pose, 'PoseStamped')
    pose_target.position.x = 0.100000
    pose_target.position.y = -0.5000000
    pose_target.position.z = 0.5000000
    pose_target.orientation.x = pose_vals[3]
    pose_target.orientation.y = pose_vals[4]
    pose_target.orientation.z = pose_vals[5]
    pose_target.orientation.w = pose_vals[6]

    group.set_goal_position_tolerance(0.0500000)
    group.set_pose_target(pose_target, end_effector_link="j2s7s300_end_effector")
    plan1 = group.plan()

    print "===== Waiting while RVIZ displays plan1 ====="
    rospy.sleep(2)
    group.execute(plan1)
    
    print "===== Plan 2 ====="
    
    group.clear_pose_targets()
    group_values = geometry_msgs.msg.Pose()
    strs = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped)
    #joint_values = group.get_current_joint_values()
    print "===== Joint values: ", strs
    #print "===== Joint values: ", joint_values
    joints = rospy.wait_for_message('/j2s7s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles)
    joints_vals = str2vals(joints, 'Joints')
    print "===== Joint values: ", joints_vals
    joints_vals[1] += 0.5
    #group.set_joint_value_target(joints_vals)
    #plan2 = group.plan()

    print "===== Waiting while RVIZ displays plan2 ====="
    #rospy.sleep(2)
    #group.execute(plan2)

    print "===== Plan 3 ====="
    waypoints = []
    values = str2vals(strs, 'PoseStamped')
    group_values.position = geometry_msgs.msg.Point(x=values[0], y=values[1], z=values[2])
    group_values.orientation = geometry_msgs.msg.Quaternion(x=values[3], y=values[4], z=values[5], w=values[6])
    waypoints.append(group_values)
    wpose = geometry_msgs.msg.Pose()
    wpose = waypoints[0]
    wpose.position.x += 0.10
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.y -= 0.20
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= 0.20
    waypoints.append(copy.deepcopy(wpose))
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

    print "===== Waiting while RVIZ displays plan3 ====="
    rospy.sleep(2)
    group.execute(plan3)
    
    rospy.sleep(0.5)
    group.set_named_target("Home")
    group.go()

    rospy.sleep(0.5)
    gripper.set_named_target("Close")
    gripper.go()

    rospy.sleep(0.5)
    gripper.set_named_target("Open")
    gripper.go()

    moveit_commander.roscpp_shutdown()
    print "===== STOPPING ====="

def str2vals(message, v_type):
    values = []
    mes_list = str(message).split('\n')
    if v_type == 'PoseStamped':    
        str_list = mes_list[8:11]
        str_list.extend(mes_list[12:])
        for index in str_list:
            values.append(float(index.split(':')[1]))
    if v_type == 'Joints':
        str_list = mes_list
        for index in str_list:
            values.append(float(index.split(':')[1]) / 180.0 * 3.1415)
    return values 

if __name__ == '__main__':
    try:
        move_group_python_interface()
    except rospy.ROSInterruptException:
        pass
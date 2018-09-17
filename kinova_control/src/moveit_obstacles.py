#! /usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Moveit_Obstacle():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_demo')
        scene = PlanningSceneInterface('world')
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        self.colors = dict()
        rospy.sleep(1)

        arm = MoveGroupCommander('arm')
        end_effector_link = arm.get_end_effector_link()
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
        arm.allow_replanning(True)
        arm.set_planning_time(5)

        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        scene.remove_world_object(table_id)
        rospy.sleep(1)

        reference_frame = 'root'

        table_ground = 0.55
        tabel_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]

        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = 0.7
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + tabel_size[2]/2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, tabel_size)

        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = 0.65
        box1_pose.pose.position.y = -0.1
        box1_pose.pose.position.z = table_ground + tabel_size[2] + box1_size[2]/2.0
        box1_pose.pose.orientation.w = 1.0
        scene.add_box(box1_id, box1_pose, box1_size)

        box2_pose = PoseStamped()
        box2_pose.header.frame_id = reference_frame
        box2_pose.pose.position.x = 0.63
        box2_pose.pose.position.y = 0.15
        box2_pose.pose.position.z = table_ground + tabel_size[2] + box2_size[2]/2.0
        box2_pose.pose.orientation.w = 1.0
        scene.add_box(box2_id, box2_pose, box2_size)

        self.setColor(table_id, 0.8, 0.0, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 1.0)
        self.sendColors()

        gripper = MoveGroupCommander('gripper')
        gripper.set_named_target('Close')
        gripper.go()
        rospy.sleep(1)

        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = 0.64
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = table_pose.pose.position.z + tabel_size[2] + 0.05
        target_quaternion = quaternion_from_euler(-1.6745735920568166, -1.2113336804234238, -1.718476017088519)
        target_pose.pose.orientation.x = target_quaternion[0]
        target_pose.pose.orientation.y = target_quaternion[1]
        target_pose.pose.orientation.z = target_quaternion[2]
        target_pose.pose.orientation.w = target_quaternion[3]       
    #    arm.set_goal_position_tolerance(0.02)
    #    arm.set_goal_orientation_tolerance(0.01)
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()
        rospy.sleep(2)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    
    def setColor(self, name, r, g, b, a=0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True

        for color in self.colors.values():
            p.object_colors.append(color)

        self.scene_pub.publish(p)

if __name__ == '__main__':
    try:
        Moveit_Obstacle()
    except KeyboardInterrupt:
        raise
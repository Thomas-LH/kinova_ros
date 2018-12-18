#! /usr/bin/env python
'''
@Description: Action server for controling kinova arm
@Author: Binghui Li
@Date: 2018-10-29 14:37:45
@LastEditTime: 2018-11-01 18:33:46
@LastEditors: Binghui Li
'''


import rospy, sys
import roslib
import actionlib
import std_msgs.msg
import kinova_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor, Grasp, PlaceLocation, GripperTranslation, MoveItErrorCodes, Constraints, OrientationConstraint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'
GRIPPER_CLOSED = [1.2, 1.2, 1.2]
GRIPPER_OPEN = [0.2, 0.2, 0.2]
GRIPPER_NEUTRAL = [0.7, 0.7, 0.7]
GRIPPER_JOINT_NAMES = ['j2s7s300_joint_finger_1', 'j2s7s300_joint_finger_2', 'j2s7s300_joint_finger_3']
GRIPPER_EFFORT = [1.0, 1.0, 1.0]
GRIPPER_FRAME = 'j2s7s300_end_effector'
REFERENCE_FRAME  = 'world'


class PPServer(object):
    """
    PPServer class manages action server and grasping functions

    Attributes:
        _scene: instance of class PlanningSceneInterface
        _scene_pub: instance of class Publisher
        colors: store objects' colors
        max_pick_attempts: max attempts for picking planning
        max_place_attempts: max attempts for placing planning
        _server: instance of class SimpleActionServer
    """

    _result = kinova_msgs.msg.PoseAndSizeResult()
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('PP_server_node')
        self._scene = PlanningSceneInterface('world')
        self._scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        self.colors = dict()
        self.max_pick_attempts = 5
        self.max_place_attempts = 5
        # initialize action server, type is PoseAndSizeAction defined in kinova_msgs/action,
        # callback function is PickAndPlace
        self._server = actionlib.SimpleActionServer("PickAndPlace", kinova_msgs.msg.PoseAndSizeAction, execute_cb=self.PickAndPlace, auto_start=False)
        self._server.start() # start action server


    '''
    Control picking and placing actions
    @param:
        goal: action goal for picking and placing
    @return: None
    '''
    def PickAndPlace(self, goal):
        arm = MoveGroupCommander('arm')
        end_effector_link = arm.get_end_effector_link() # get end effector link name
        arm.allow_replanning(True)
        arm.set_planning_time(5)
        # setting object id
        table_id = 'table'
        side_id = 'side'
        table2_id = 'table2'
        table2_ground_id = 'table2_ground'
        box2_id = 'box2'
        target_id = 'target'
        wall_id = 'wall'
        ground_id = 'ground'
        rospy.sleep(1)
        # setting object size
        table_ground = 0.55
        table_size = [0.7, 0.2, 0.01]
        table2_size = [0.6, 0.25, 0.01]
        table2_ground_size = [0.9, 0.02, 0.6]
        box2_size = [0.05, 0.05, 0.15]
        wall_size = [0.9, 0.01, 0.6]
        ground_size = [3, 3, 0.01]
        side_size = [0.7, 0.01, table_ground]
        # setting object pose and add to the world
        '''
        self.scene_manage(table2_ground_id, table2_ground_size, [ 0.0, 0.36, table2_ground_size[2]/2.0])
        self.scene_manage(box2_id, box2_size, [-0.12, -0.63, table_ground + table_size[2] + box2_size[2]/2.0])
        self.scene_manage(table_id, table_size, [0.0, -0.7, table_ground + table_size[2]/2.0])
        self.scene_manage(table2_id, table2_size, [0.0, 0.325, table2_ground_size[2] + table2_size[2]/2.0])
        self.scene_manage(wall_id, wall_size, [0.0, 0.405, table2_ground_size[2] + table2_size[2] + wall_size[2]/2.0])
        self.scene_manage(ground_id, ground_size, [0.0, 0.0, -ground_size[2]/2.0])
        self.scene_manage(side_id, side_size, [0.0, -0.605, side_size[2]/2.0])
        '''
        self.scene_manage(ground_id, ground_size, [0.0, 0.0, -ground_size[2]/2.0])
        target_size = [goal.object_size.x, goal.object_size.y, goal.object_size.z]
        target_position = [goal.object_pose.pose.position.x, goal.object_pose.pose.position.y, goal.object_pose.pose.position.z]
        self.scene_manage(target_id, target_size, target_position)

        # setting object color
        self.setColor(table_id, 0.8, 0.0, 0.0, 1.0)
        self.setColor(table2_id, 0.8, 0.0, 0.0)
        self.setColor(table2_ground_id, 0.9, 0.9, 0.9)
        self.setColor(box2_id, 0.8, 0.4, 1.0)
        self.setColor(target_id, 0.5, 0.4, 1.0)
        self.setColor(wall_id, 0.9, 0.9, 0.9)
        self.setColor(ground_id, 0.3, 0.3, 0.3, 1.0)
        self.setColor(side_id, 0.8, 0.0, 0.0, 1.0)
        self.sendColors()

        arm.set_support_surface_name(ground_id) # avoid collision detection

        # setting placing position
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_orientation = Quaternion()
        place_orientation = quaternion_from_euler(0.0, 0.0, 0.0)
    #    self.setPose(place_pose, [-0.22, 0.24, table2_ground_size[2] + table2_size[2] + target_size[2]/2.0], list(place_orientation))
        self.setPose(place_pose, [0.0, -0.5, 0.25 + target_size[2]/2.0])
        # setting grasping position 
        grasp_pose = goal.object_pose
        grasp_init_orientation = Quaternion()
        grasp_init_orientation = quaternion_from_euler(1.57, -1.57, 0.0)  
        grasp_pose.pose.orientation.x = grasp_init_orientation[0]
        grasp_pose.pose.orientation.y = grasp_init_orientation[1]
        grasp_pose.pose.orientation.z = grasp_init_orientation[2]
        grasp_pose.pose.orientation.w = grasp_init_orientation[3]
        grasp_pose.pose.position.y += 0.02
        rospy.loginfo('quaterion: '+ str(grasp_pose.pose.orientation))
        # generate grasp postures
        grasps = self.make_grasps(grasp_pose, [table_id], [0.05, 0.07, [0.0, -1.0, 0.0]], [0.1, 0.15, [0.0, 1.0, 1.0]], 1)

        result = None
        n_attempts = 0
        # execute pick action 
        while result != MoveItErrorCodes.SUCCESS and n_attempts < self.max_pick_attempts:
            result = arm.pick(target_id, grasps)
            n_attempts += 1
            rospy.loginfo('Pick attempt:' + str(n_attempts))
            rospy.sleep(0.2)

        arm.set_support_surface_name(ground_id)
        # generate place action
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            places = self.make_places(place_pose, [table2_id], [0.05, 0.07, [0.0, -1.0, -1.0]], [0.1, 0.15, [0.0, 1.0, 1.0]])
        # execute place action
        while result != MoveItErrorCodes.SUCCESS and n_attempts < self.max_place_attempts:
            for place in places:
                result = arm.place(target_id, place)
                if result == MoveItErrorCodes.SUCCESS:
                    break

            n_attempts += 1
            rospy.loginfo('Place attempt:' + str(n_attempts))
            rospy.sleep(0.2)
        # return result and restore Home posture
        if MoveItErrorCodes.SUCCESS:
            arm.set_named_target('Home')
            arm.go()
            #rospy.Subscriber('/j2s7s300_driver/out/tool_pose', PoseStamped, callback=self.cb)
            #self._result.arm_pose = arm.get_current_pose()
            self._result.arm_pose = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', PoseStamped)
            self._server.set_succeeded(self._result)

        rospy.sleep(2)
    #    moveit_commander.roscpp_shutdown()
    #    moveit_commander.os._exit(0)

    '''
    Setting object's size and position, then adding them to scene 
    @param:
        obj_id: ID of object
        obj_size: size of object
        obj_pose: position of object 
    @return: None
    '''
    def scene_manage(self, obj_id, obj_size, obj_pose):
        self._scene.remove_world_object(obj_id)
        obj_pos = PoseStamped()
        obj_pos.header.frame_id = REFERENCE_FRAME
        self.setPose(obj_pos, obj_pose)    
        self._scene.add_box(obj_id, obj_pos, obj_size)

    '''
    Setting posestamped object
    @param:
        pose_stamped_object: posestamped object
        position: positon of object
        orientation: orientation of object
    @return: 
    '''
    def setPose(self, pose_stamped_object, position, orientation=None):
        if type(pose_stamped_object) is not PoseStamped:
            raise Exception('Parameter pose_stamped_object must be a PoseStamped object')
        pose_stamped_object.pose.position.x = position[0]
        pose_stamped_object.pose.position.y = position[1]
        pose_stamped_object.pose.position.z = position[2]

        if orientation is None:
            pose_stamped_object.pose.orientation.w = 1.0
        else:
            pose_stamped_object.pose.orientation.x = orientation[0]
            pose_stamped_object.pose.orientation.y = orientation[1]
            pose_stamped_object.pose.orientation.z = orientation[2]
            pose_stamped_object.pose.orientation.w = orientation[3] 
    
    '''
    Setting colors of object 
    @param:
        name: ID of object
        r, g, b, a: color and transparency of object
    @return: None
    '''
    def setColor(self, name, r, g, b, a=0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    '''
    Sending colors to scene
    @return: None
    '''
    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True

        for color in self.colors.values():
            p.object_colors.append(color)

        self._scene_pub.publish(p)

    '''
    Generating grasp postures
    @param:
        initial_pose_stamped: initial stamped grsap pose
        allowed_touch_objects: objects which do not need collision detection
        pre: pre-grasp distance and orientation
        post: retreat distance and orientation
    @return: 
        grasp postures
    '''
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects, pre, post, set_rpy = 0):
        g = Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
        g.pre_grasp_approach = self.make_gripper_translation(pre[0], pre[1], pre[2])
        g.post_grasp_retreat = self.make_gripper_translation(post[0], post[1], post[2])

        g.grasp_pose = initial_pose_stamped
        roll_vals = [1.57]
        yaw_vals = [0, 0.2, -0.2, 0.4, -0.4, 0.6, -0.6]
        pitch_vals = [-1.57, -1.47, -1.67, -1.37, -1.77]
        z_vals =[0]

        grasps = []

        for y in yaw_vals:
            for p in pitch_vals:
                for z in z_vals:
                    for r in roll_vals:
                        if set_rpy:
                            q = quaternion_from_euler(r, p, y)
                            g.grasp_pose.pose.orientation.x = q[0]
                            g.grasp_pose.pose.orientation.y = q[1]
                            g.grasp_pose.pose.orientation.z = q[2]
                            g.grasp_pose.pose.orientation.w = q[3]

                        g.grasp_pose.pose.position.z = initial_pose_stamped.pose.position.z + z

                        g.id = str(len(grasps))
                        g.allowed_touch_objects = allowed_touch_objects
                        g.max_contact_force = 0
                        g.grasp_quality = 1.0 - abs(p)
                        grasps.append(deepcopy(g))

        return grasps

    '''
    Setting grasp posture
    @param:
        joint_positions: posture for fingers 
    @return: 
        JointTrajectory object
    '''
    def make_gripper_posture(self, joint_positions):
        t = JointTrajectory()
        t.joint_names = GRIPPER_JOINT_NAMES

        tp = JointTrajectoryPoint()
        tp.positions = joint_positions
        tp.effort = GRIPPER_EFFORT
        t.points.append(tp)

        return t

    '''
    Setting gripper translation
    @param:
        min_dist: min distance of pre_posture
        desired: desired distance of pre_posture
        vector: orientation vector for translation
    @return: 
        GripperTranslation object
    '''
    def make_gripper_translation(self, min_dist, desired, vector):
        g = GripperTranslation()
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        g.direction.header.frame_id = REFERENCE_FRAME   
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    '''
    Generating placing postures
    @param:
        init_pose: initial stamped place pose
        allowed_touch_objects: objects which do not need collision detection
        pre: pre-place distance and orientation
        post: retreat distance and orientation
        set_rpy: flag for setting roll, pitch, yaw
        roll_vals, pitch_vals, yaw_vals: orientation of target
    @return: 
        place postures
    '''
    def make_places(self, init_pose, allowed_touch_objects, pre, post, set_rpy = 0,roll_vals=[0], pitch_vals=[0], yaw_vals=[0]):
        place = PlaceLocation()
        place.post_place_posture = self.make_gripper_posture(GRIPPER_OPEN)
        place.pre_place_approach = self.make_gripper_translation(pre[0], pre[1], pre[2])
        place.post_place_retreat = self.make_gripper_translation(post[0], post[1], post[2])

        place.place_pose = init_pose 
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]

        places = []

        for yaw in yaw_vals:
            for pitch in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.place_pose.pose.position.x = init_pose.pose.position.x + x
                        place.place_pose.pose.position.y = init_pose.pose.position.y + y

                        q = quaternion_from_euler(0, pitch, yaw)

                        if  set_rpy:
                            place.place_pose.pose.orientation.x = q[0]
                            place.place_pose.pose.orientation.y = q[1]
                            place.place_pose.pose.orientation.z = q[2]
                            place.place_pose.pose.orientation.w = q[3]
                        place.id = str(len(places))
                        place.allowed_touch_objects = allowed_touch_objects

                        places.append(deepcopy(place))
        return places

if __name__ == '__main__':
    try:
        PPServer()
        rospy.spin()
    except KeyboardInterrupt:
        raise
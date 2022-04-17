#!/usr/bin/env python

from __future__ import print_function

import geometry_msgs.msg
import moveit_commander
import rospy
import numpy as np
import tf
import tf2_ros
import dynamic_reconfigure.client
from tf2_py import TransformException

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import PlanningSceneComponents, PlanningScene, PlanningSceneWorld, CollisionObject
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene, ApplyPlanningSceneRequest
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction
from tf.transformations import euler_from_quaternion, euler_matrix
from pick_up_object.utils import to_frame_pose, PlanningSceneInterface

class ArmTorsoController:
    def __init__(self):
        self._listener = tf.TransformListener()
        rospy.sleep(3)
        self._robot = moveit_commander.RobotCommander()
        # self._group = moveit_commander.MoveGroupCommander('')
        group_name = "arm_torso"
        if not self._robot.has_group(group_name):
            raise RuntimeError("the move_group [arm_torso] is not found")
        self._move_group = self._robot.get_group(group_name)
        print(self._move_group)

        self._joints = self._move_group.get_joints()
        self._scene = PlanningSceneInterface()
        self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.debug_pose = rospy.Publisher('debug_plan_pose', PoseStamped, queue_size=1, latch=True)
        self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene, queue_size=1)
        self._pubPlanningSceneWorld = rospy.Publisher('planning_scene_world', PlanningSceneWorld, queue_size=1)
        
        rospy.wait_for_service('/apply_planning_scene', 10.0)
        self._apply_planning_scene_diff = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        
        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        
        self.plan_client = dynamic_reconfigure.client.Client('/move_group/plan_execution', timeout=10)
        
        self.pm_client = SimpleActionClient('/play_motion', PlayMotionAction)
        self.pm_client.wait_for_server()

        

    @property
    def robot(self):
        return self._robot

    @property
    def scene(self):
        return self._scene

    @property
    def move_group(self):
        return self._move_group

    def get_ee_map_pose(self):
        while True:
            try:
                def _round(value):
                    return round(value, 3)

                trans, quaternion = self._listener.lookupTransform('/map', '/arm_tool_link', rospy.Time(0))

                return tuple(map(_round, trans)), tuple(map(_round, quaternion))
            except TransformException:
                rospy.sleep(0.1)

    @property
    def joints(self):
        return tuple(self._joints)

    def get_joint_values(self):
        return self._move_group.get_current_joint_values()

    def configure_planner(self, config=dict()):
        """
        Arguments:
            config {dict} -- Dictionary to set configuration for moveit motion planner of this move group 
        """
        planner_id = config.get('planner_id', 'RRTConnectkConfigDefault')
        planning_time = config.get('planning_time', 10)
        num_planning_attempts = config.get('num_planning_attempts', 10)
        eef_link = config.get('eef_link', 'gripper_grasping_frame')
        allow_replanning = config.get('allow_replanning', True)
        goal_pos_tol = config.get('goal_pos_tol', 0.001)
        goal_orien_tol = config.get('goal_orien_tol', 0.001)
        goal_joint_tol = config.get('goal_joint_tolerance', 0.001)
        max_velocity = config.get('max_velocity', 0.9)
        max_acceleration = config.get('max_acceleration', 0.4)
        pose_frame = config.get('pose_frame', None)

        # self.plan_client.update_configuration({'max_replan_attempts': num_planning_attempts})

        self._move_group.set_planner_id(planner_id)
        self._move_group.set_planning_time(planning_time)
        self._move_group.set_num_planning_attempts(num_planning_attempts)
        self._move_group.set_end_effector_link(eef_link)
        self._move_group.allow_replanning(allow_replanning)
        self._move_group.set_max_velocity_scaling_factor(max_velocity)
        self._move_group.set_max_acceleration_scaling_factor(max_acceleration)
        if goal_orien_tol:
            self._move_group.set_goal_orientation_tolerance(goal_orien_tol)
        if goal_pos_tol:
            self._move_group.set_goal_position_tolerance(goal_pos_tol)
        if goal_joint_tol:
            self._move_group.set_goal_joint_tolerance(goal_joint_tol)
        if pose_frame:
            self._move_group.set_pose_reference_frame(pose_frame)

    def sync_shift_ee_frame(self, shift_frame, x=0., y=0., z=0.):
        """
        Arguments:
            shift_frame {frame id} -- frame id in which to shift end-effector
            x, y, z {axes} -- direction in which to shift

        Return:
            bool -- result from motion
        """

        try:
            curr_pose = self._move_group.get_current_pose()
            curr_pose.pose = to_frame_pose(curr_pose.pose, source_frame=curr_pose.header.frame_id, target_frame=shift_frame)
            curr_pose.header.frame_id = shift_frame

            curr_pose.pose.position.x += x
            curr_pose.pose.position.y += y
            curr_pose.pose.position.z += z

            self._move_group.set_pose_target(curr_pose)

            # publish pose for debugging purposes
            # print('Publishing debug_plan_pose')
            self.debug_pose.publish(curr_pose)

            self._move_group.set_start_state_to_current_state()
            result = self._move_group.go(wait=True)
            return result
        except Exception as e:
            print('Failed: sync_shift_ee_frame: {}'.format(e))
            return False

    def sync_shift_ee(self, x=0., y=0., z=0.):
        """
        Arguments:
            x, y, z -- direction in the gripper_grasping_frame
        Return:
            bool -- result from motion
        """

        try:
            curr_pose = self._move_group.get_current_pose()
            pose = curr_pose.pose
            # print('x: {}, y: {}, z: {}'.format(pose.position.x, pose.position.y, pose.position.z))

            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            euler = euler_from_quaternion(quat)
            euler_m = euler_matrix(*euler)

            # calculate offset to add to the current pose
            # delta = np.matmul(euler_m[:3, :3], np.array([x, y, z]).T)
            delta = np.dot(euler_m[:3, :3], np.array([x, y, z]).T)
            pose.position.x += delta[0]
            pose.position.y += delta[1]
            pose.position.z += delta[2]

            self._move_group.set_pose_target(curr_pose)
            # publish pose for debugging purposes
            # print('Publishing debug_plan_pose')
            self.debug_pose.publish(curr_pose)

            self._move_group.set_start_state_to_current_state()
            result = self._move_group.go(wait=True)
            return result
        except Exception as e:
            print('Failed: sync_shift_ee: {}'.format(e))
            return False
    
    def sync_reach_ee_pose(self, position, orientation):
        """
        Arguments:
            position {x, y, z} -- pose
            orientation {x, y, z, w} -- quaternion

        Return:
            bool -- result from motion
        """

        try:
            # type: (Sequence[float,float,float], Sequence[float,float,float,float]) -> bool
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = position[0]
            pose_goal.position.y = position[1]
            pose_goal.position.z = position[2]
            pose_goal.orientation.x = orientation[0]
            pose_goal.orientation.y = orientation[1]
            pose_goal.orientation.z = orientation[2]
            pose_goal.orientation.w = orientation[3]
            self._move_group.set_pose_target(pose_goal)
            self._move_group.set_start_state_to_current_state()
            result = self._move_group.go(wait=True)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            return result
        except Exception as e:
            print('Failed: sync_reach_ee_pose: {}'.format(e))
            return False
    
    def sync_reach_ee_poses(self, posearray):
        """
        Arguments:
            posearray {PoseArray} -- array of poses as a goal for moveit
        
        Return:
            bool -- result from motion
        """

        try:
            self._move_group.set_pose_reference_frame(posearray.header.frame_id)
            self._move_group.set_pose_targets(posearray.poses)

            self._move_group.set_start_state_to_current_state()
            result = self._move_group.go(wait=True)
            return result
        except Exception as e:
            print('Failed: sync_reach_ee_poses: {}'.format(e))
            return False

    def sync_reach_safe_joint_space(self):
        """
            Predefined pose with arm and torso stretched
        """

        return self.sync_reach_joint_space(0.35, (0.07, 0.92, 0.16, 0.85, -1.62, 0.03, 1.74))

    def sync_reach_joint_space(self, torso_goal=None, arm_goals=None):
        """
        Arguments:
            torso_goal -- torso joint value
            arm_goals -- arm joints values
        Return:
            bool -- result from motion goal
        """

        if arm_goals is None:
            arm_goals = []

        current_state = self._move_group.get_current_joint_values()
        joint_goal = current_state[:]
        if torso_goal is not None:
            joint_goal[0] = torso_goal

        arm_joint_count = len(joint_goal) - 1
        for i in range(arm_joint_count):
            if i >= len(arm_goals):
                break
            if arm_goals[i] is not None:
                joint_goal[i + 1] = arm_goals[i]

        result = False
        if joint_goal != current_state:
            # self._move_group.set_max_velocity_scaling_factor(1.0)
            result = self._move_group.go(joint_goal, wait=True)
            self._move_group.stop()
        return result

    def update_planning_scene(self, add=True, entry_names=['object']):
        """
        Arguments:
            add {bool} -- To add object to the collision matrix from the planning scene            
        """

        request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        
        scene = self.get_planning_scene(request).scene
        acm = scene.allowed_collision_matrix
        print(acm.default_entry_names)
        print('received entry names {}'.format(entry_names))

        if add and not 'object' in acm.default_entry_names:
            acm.default_entry_names += entry_names
            acm.default_entry_values += [True for name in entry_names]
            # new_planning_scene = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
            # self._pubPlanningScene.publish(planning_scene_diff)
            # self.apply_planning_scene(planning_scene_diff)

            scene.is_diff=True
            scene.allowed_collision_matrix=acm
            apply_planning_scene_req = ApplyPlanningSceneRequest()
            apply_planning_scene_req.scene = scene
            self._apply_planning_scene_diff(apply_planning_scene_req)
            print(acm.default_entry_names)
            print('Updated Scene plan: Added')
            rospy.sleep(2.0)
        elif not add:
            acm.default_entry_names = []
            acm.default_entry_values = []
            planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
            self._pubPlanningScene.publish(planning_scene_diff)
            print('Updated Scene plan: Removed')
            rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node("arm_test", anonymous=True)
    _arm = ArmTorsoController()
    print(_arm.robot.get_joint_names())
    # _arm.sync_reach_joint_space(0.35, (0.07, 0.92, 0.16, 0.85, -1.62, 0.03, 0.46))

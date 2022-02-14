from __future__ import print_function

import time

import geometry_msgs.msg
import moveit_commander
import rospy
import numpy as np
import tf
from tf2_py import TransformException
from typing import List, Tuple, Optional, Sequence
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import PlanningSceneComponents, PlanningScene, PlanningSceneWorld, CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_place_object.msg import *
from std_msgs.msg import Header
import dynamic_reconfigure.client
from contact_graspnet.utilities.transformations import *

class ArmTorsoController:
    _joints = None  # type: List[str]

    def __init__(self):
        self._listener = tf.TransformListener()
        self._robot = moveit_commander.RobotCommander()
        # self._group = moveit_commander.MoveGroupCommander('')
        group_name = "arm_torso"
        if not self._robot.has_group(group_name):
            raise RuntimeError("the move_group [arm_torso] is not found")
        self._move_group = self._robot.get_group(group_name)

        self._joints = self._move_group.get_joints()
        self._scene = moveit_commander.PlanningSceneInterface()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.debug_pose = rospy.Publisher('debug_plan_pose', PoseStamped, queue_size=1, latch=True)
        self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene, queue_size=1)
        self._pubPlanningSceneWorld = rospy.Publisher('planning_scene_world', PlanningSceneWorld, queue_size=1)
        rospy.wait_for_service('/apply_planning_scene', 10.0)
        self.apply_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
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
        # type: () -> Tuple[Sequence[float],Sequence[float]]
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
        # type: () -> Tuple[str]
        return tuple(self._joints)

    def get_joint_values(self):
        # type: () -> List[float]
        return self._move_group.get_current_joint_values()

    def configure_planner(self, config=dict()):
        planner_id = config.get('planner_id', 'RRTConnectkConfigDefault')
        planning_time = config.get('planning_time', 10)
        num_planning_attempts = config.get('num_planning_attempts', 3)
        planning_attempts = config.get('planning_attempts', 3)
        eef_link = config.get('eef_link', 'gripper_grasping_frame')
        allow_replanning = config.get('allow_replanning', True)
        goal_pos_tol = config.get('goal_pos_tol', 0.001)
        goal_orien_tol = config.get('goal_orien_tol', 0.0001)
        max_velocity = config.get('max_velocity', 0.3)
        max_acceleration = config.get('max_acceleration', 0.4)
        pose_frame = config.get('pose_frame', None)

        self.plan_client.update_configuration({'max_replan_attempts': planning_attempts})

        self._move_group.set_planner_id(planner_id)
        self._move_group.set_planning_time(planning_time)
        self._move_group.set_num_planning_attempts(num_planning_attempts)
        self._move_group.set_end_effector_link(eef_link)
        self._move_group.allow_replanning(allow_replanning)
        self._move_group.set_max_velocity_scaling_factor(max_velocity)
        self._move_group.set_max_acceleration_scaling_factor(max_acceleration)
        if goal_pos_tol:
            self._move_group.set_goal_position_tolerance(goal_pos_tol)
        if goal_orien_tol:
            self._move_group.set_goal_orientation_tolerance(goal_orien_tol)
        if pose_frame:
            self._move_group.set_pose_reference_frame(pose_frame)

    def sync_shift_ee_frame(self, shift_frame, x=0., y=0., z=0.):
        try:
            curr_pose = self._move_group.get_current_pose()
            curr_pose.pose = self.to_frame_pose(curr_pose.pose, source_frame=curr_pose.header.frame_id, target_frame=shift_frame)
            curr_pose.header.frame_id = shift_frame

            curr_pose.pose.position.x += x
            curr_pose.pose.position.y += y
            curr_pose.pose.position.z += z

            self._move_group.set_pose_target(curr_pose)
            self.debug_pose.publish(curr_pose)
            print('Published debug_plan_pose')

            self._move_group.set_start_state_to_current_state()
            result = self._move_group.go(wait=True)
            return result
        except Exception as e:
            print('Failed: sync_shift_ee_frame: {}'.format(e))
            return False

    def sync_shift_ee(self, x=0., y=0., z=0.):
        try:
            # if len(shift_frame) == 0:
            #     shift_frame = self.move_group.get_end_effector_link()

            # self._move_group.set_max_velocity_scaling_factor(0.3)

            curr_pose = self._move_group.get_current_pose()
            pose = curr_pose.pose
            # print('x: {}, y: {}, z: {}'.format(pose.position.x, pose.position.y, pose.position.z))

            q_orien = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            euler = euler_from_quaternion(q_orien)
            euler_m = euler_matrix(*euler)

            delta = np.matmul(euler_m[:3, :3], np.array([x, y, z]).T)
            pose.position.x += delta[0]
            pose.position.y += delta[1]
            pose.position.z += delta[2]

            # print('x: {}, y: {}, z: {}'.format(pose.position.x, pose.position.y, pose.position.z))

            # eef_pose = self.to_frame_pose(curr_pose.pose, source_frame=curr_pose.header.frame_id, target_frame=shift_frame)
            # eef_pose.position.x += x
            # eef_pose.position.y += y
            # eef_pose.position.z += z
            # base_pose = self.to_frame_pose(eef_pose, source_frame=shift_frame, target_frame=curr_pose.header.frame_id)
            # # base_pose.orientation = curr_pose.pose.orientation # TODO: HACK why is it coming back negative?

            # base_pose_stamped = PoseStamped()
            # base_pose_stamped.header.frame_id = curr_pose.header.frame_id
            # base_pose_stamped.header.stamp = rospy.get_rostime()
            # base_pose_stamped.pose = base_pose
            self._move_group.set_pose_target(curr_pose)
            self.debug_pose.publish(curr_pose)
            print('Published debug_plan_pose')

            self._move_group.set_start_state_to_current_state()
            result = self._move_group.go(wait=True)
            return result
        except Exception as e:
            print('Failed: sync_shift_ee: {}'.format(e))
            return False
    
    def sync_reach_ee_pose(self, position, orientation):
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
        try:
            self._move_group.set_pose_reference_frame(posearray.header.frame_id)
            self._move_group.set_pose_targets(posearray.poses)

            self._move_group.set_start_state_to_current_state()
            result = self._move_group.go(wait=True)
            return result
        except Exception as e:
            print('Failed: sync_reach_ee_poses: {}'.format(e))
            print(len(posearray.poses))
            return False

    def sync_reach_ee_poses_old(self, posearray):
        # type: (Sequence[float,float,float], Sequence[float,float,float,float]) -> bool
        self._move_group.set_start_state_to_current_state()
        self._move_group.allow_replanning(True)
        print(self._move_group.get_planner_id())
        # self._move_group.set_planner_id('SBLkConfigDefault')
        self._move_group.set_planner_id('RRTConnectkConfigDefault')
        # self._move_group.set_planner_id('BKPIECEkConfigDefault')
        self._move_group.set_pose_reference_frame(posearray.header.frame_id)
        # print(self._move_group.get_pose_reference_frame())
        self._move_group.set_end_effector_link('gripper_grasping_frame')
        print('End Effector: ' + self._move_group.get_end_effector_link())

        # self._move_group.set_goal_position_tolerance(0.0005)
        self._move_group.set_goal_position_tolerance(0.001)
        print(self._move_group.get_goal_tolerance())

        self._move_group.set_pose_targets(posearray.poses)

        print('About to Move to Approach')
        result = self._move_group.go(wait=True)
        if result:
            # self._scene.remove_world_object('object')
            self.update_planning_scene(add=True)
    
            pose = self._move_group.get_current_pose(end_effector_link='gripper_grasping_frame')
            gripper_pose = self.to_frame_pose(pose.pose, source_frame=pose.header.frame_id, target_frame='gripper_grasping_frame')
            gripper_pose.position.x += 0.05
            base_pose = self.to_frame_pose(gripper_pose, source_frame='gripper_grasping_frame', target_frame=pose.header.frame_id)
            self._move_group.set_pose_target(base_pose, end_effector_link='gripper_grasping_frame')

            base_pose_stamped = PoseStamped()
            base_pose_stamped.header.frame_id = pose.header.frame_id
            base_pose_stamped.header.stamp = rospy.get_rostime()
            base_pose_stamped.pose = base_pose
            self.debug_pose.publish(base_pose_stamped)
            print('Published debug_plan_pose')

            print('About to Shift Forward')
            result = self._move_group.go(wait=True)
            print('result from shift forward: {}'.format(result))

            if result:
                print('About to Play Motion')
                result = self.play_motion_action('close')

                if result:
                    # before the object is attached, remove it from the world
                    self._scene.remove_world_object('object')
                    self._move_group.attach_object('object', link_name="gripper_grasping_frame", touch_links=['gripper_left_finger_link', 'gripper_right_finger_link'])

                    print('About to Lift')
                    pose = self._move_group.get_current_pose(end_effector_link='gripper_grasping_frame')
                    pose.pose.position.z += 0.15
                    self._move_group.set_pose_target(pose, end_effector_link='gripper_grasping_frame')
                    result = self._move_group.go(wait=True)

                    goal = PlaceObjectGoal(storage='tray', attached_name="")
                    self.place_obj_client.send_goal(goal)
                    self.place_obj_client.wait_for_result()
                    print(self.place_obj_client.get_result())
        
        self.update_planning_scene(add=False)
        self._move_group.clear_pose_targets()
        self._move_group.stop()
        return result

    def sync_reach_safe_joint_space(self):
        return self.sync_reach_joint_space(0.35, (0.07, 0.92, 0.16, 0.85, -1.62, 0.03, 0.46))

    def sync_reach_joint_space(self, torso_goal=None, arm_goals=None):
        # type: (Optional[float], Optional[Sequence[float]]) -> bool
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

    def play_motion_action(self, action='home'):
        goal = PlayMotionGoal()
        goal.motion_name = action
        goal.skip_planning = False
        goal.priority = 0  # Optional

        print("Sending goal with motion: " + action)
        self.pm_client.send_goal(goal)

        print("Waiting for result...")
        action_ok = self.pm_client.wait_for_result(rospy.Duration(30.0))

        state = self.pm_client.get_state()

        return action_ok

    def update_planning_scene(self, add=True):
        request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = self.get_planning_scene(request)
        acm = response.scene.allowed_collision_matrix
        print(acm.default_entry_names)
        if add and not 'object' in acm.default_entry_names:
            acm.default_entry_names += ['object']
            acm.default_entry_values += [True]
            planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
            self._pubPlanningScene.publish(planning_scene_diff)
            # self.apply_planning_scene(planning_scene_diff)
            print('Updated Scene plan: Added')
            rospy.sleep(2.0)
        elif not add:
            acm.default_entry_names = []
            acm.default_entry_values = []
            planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
            self._pubPlanningScene.publish(planning_scene_diff)
            print('Updated Scene plan: Removed')
            rospy.sleep(2.0)
    
    def update_planning_scene_world(self):
        planning_scene = PlanningSceneWorld()
        print(self._scene.get_objects(['object']))

    def to_frame_pose(self, pose, source_frame='xtion_depth_optical_frame', target_frame='base_footprint'):
        """
            Converts to target frame
            Returns the pose in the target frame
        """
        # remove '/' from source_frame and target frame to avoid tf2.InvalidArgumentException
        source_frame = source_frame.replace('/', '')
        target_frame = target_frame.replace('/', '')
        try:
            transformation = self.tfBuffer.lookup_transform(target_frame, source_frame,
            rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
        
        pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=pose), transformation).pose
        return pose

if __name__ == '__main__':
    rospy.init_node("arm_test", anonymous=True)
    _arm = ArmTorsoController()
    print(_arm.robot.get_joint_names())
    # _arm.sync_reach_joint_space(0.35, (0.07, 0.92, 0.16, 0.85, -1.62, 0.03, 0.46))

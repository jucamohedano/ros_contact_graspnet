#!/usr/bin/python
import rospy
import smach
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import dynamic_reconfigure.client

from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point, PoseArray, Vector3, PointStamped
from sensor_msgs.msg import PointCloud2
from pick_up_object.utils import clear_octomap
from std_msgs.msg import Header
from pick_up_object.utils import tf_transform
from moveit_msgs.msg import AttachedCollisionObject



class Pickup(smach.State):
    
    def __init__(self, planning_scene, arm_torso_controller):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['objs_resp', 'grasps_resp'],
                             output_keys=['prev']
                             )
        self.planning_scene = planning_scene
        self.arm_torso_controller = arm_torso_controller
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


    def execute(self, userdata):
        userdata.prev = 'Pickup'
        objs_resp = userdata.objs_resp
        grasps_resp = userdata.grasps_resp

        result = self.pick_object(objs_resp, grasps_resp.all_grasp_poses[0], 0)

    def pick_object(self, objs_resp, grasp_poses, obj_idx):

        eef_link = self.arm_torso_controller.move_group.get_end_effector_link()

        # remove any previous objects added to the planning scene
        self.planning_scene.remove_attached_object(eef_link, name='object')
        self.planning_scene.remove_world_object('object')

        co = self.add_collision_object(objs_resp.object_clouds[0])
        clear_octomap()
        rospy.sleep(3.0)

        config = dict(planning_time=3., allow_replanning=True)
        self.arm_torso_controller.configure_planner(config)

        self.arm_torso_controller.sync_reach_safe_joint_space()
        self.arm_torso_controller.play_motion_action('open')

        print('About to Approach')
        all_grasps_base = tf_transform(grasp_poses, 'base_footprint')
        # self.target_poses_pub.publish(all_grasps_base)
        result = self.arm_torso_controller.sync_reach_ee_poses(all_grasps_base)

        rospy.sleep(5.0)

        if not result:
            print('Approach Failed')
            # home_result = self.arm_torso_controller.play_motion_action('home')
            self.arm_torso_controller.sync_reach_safe_joint_space()
        else:
            print('About to Shift Forward')
            # self.planning_scene.remove_world_object('object')
            self.arm_torso_controller.update_planning_scene(add=True)

            shift = 0.08+0.1
            config['planning_attempts'] = 1
            self.arm_torso_controller.configure_planner(config)
            
            for i in range(3):
                print('Shift Try: {}'.format(i + 1))
                result = self.arm_torso_controller.sync_shift_ee(x=shift)
                if result:
                    break
                else:
                    shift -= 0.025

            del config['planning_attempts']
            self.arm_torso_controller.configure_planner(config)
            

            rospy.sleep(5.)

            if not result:
                print('Shift Forward Failed')
                # co = self.add_collision_object(objs_resp.object_clouds[obj_ind])
                # home_result = self.arm_torso_controller.play_motion_action('home')
                self.arm_torso_controller.sync_reach_safe_joint_space()
                self.planning_scene.remove_world_object('object')
            else:
                print('About to Close')
                result = self.arm_torso_controller.play_motion_action('close')

                rospy.sleep(5.)

                if not result:
                    print('Close Failed')
                    co = self.add_collision_object(objs_resp)
                    # home_result = self.arm_torso_controller.play_motion_action('home')
                    self.arm_torso_controller.sync_reach_safe_joint_space()
                    self.planning_scene.remove_world_object('object')
                else:
                    print('Attaching Object')
                    aco = AttachedCollisionObject()
                    aco.link_name = self.arm_torso_controller.move_group.get_end_effector_link()
                    aco.object = co
                    aco.touch_links = ['gripper_link', 'gripper_left_finger_link', 'gripper_right_finger_link']
                    self.planning_scene.attach_object(aco)

                    rospy.sleep(5.)

                    print('About to Lift')
                    result = self.arm_torso_controller.sync_shift_ee_frame(z=0.2, shift_frame='map')  

                    rospy.sleep(5.)

                    if not result:
                        print('Lift Failed')
                        # home_result = self.arm_torso_controller.play_motion_action('home')
                        self.arm_torso_controller.sync_reach_safe_joint_space()
                    else:
                        self.planning_scene.remove_attached_object(eef_link, name='object')
                        self.planning_scene.remove_world_object('object')

                        print('About to Open')
                        result = self.arm_torso_controller.play_motion_action('open')

                        rospy.sleep(5.)

                        if not result:
                            print('Open Failed')
                            # home_result = self.arm_torso_controller.play_motion_action('home')
                            self.arm_torso_controller.sync_reach_safe_joint_space()
                        else:
                            print('Open Succeeded')
                            # home_result = self.arm_torso_controller.play_motion_action('home')
                            self.arm_torso_controller.sync_reach_safe_joint_space()


        self.planning_scene.remove_attached_object(eef_link, name='object')
        self.planning_scene.remove_world_object('object')
        # self.psm_client.update_configuration({'publish_geometry_updates': True })
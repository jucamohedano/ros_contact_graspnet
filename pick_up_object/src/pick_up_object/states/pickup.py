#!/usr/bin/python

import rospy
import smach
import numpy as np
# import dynamic_reconfigure.client
import moveit_commander

from moveit_msgs.msg import AttachedCollisionObject
from geometry_msgs.msg import PoseArray
from pick_up_object.utils import play_motion_action, add_collision_object, clear_octomap

# TODO: create a substate machine with the code from pick_object function

class Pickup(smach.State):
    
    def __init__(self, arm_torso_controller):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['objs_resp', 'grasps_resp'],
                             output_keys=['prev']
                             )
        self.arm_torso_controller = arm_torso_controller
        self.planning_scene = arm_torso_controller._scene
        self.target_poses_pub = rospy.Publisher('/current_target_poses', PoseArray, queue_size=200, latch=True)
        # self.psm_client = dynamic_reconfigure.client.Client('/move_group/planning_scene_monitor', timeout=10)


    def execute(self, userdata):
        userdata.prev = 'Pickup'
        objs_resp = userdata.objs_resp
        grasps_resp = userdata.grasps_resp

        result = self.pick_object(objs_resp, grasps_resp.all_grasp_poses[0], 0)

        return result

    def pick_object(self, objs_resp, grasp_poses, obj_idx):

        eef_link = self.arm_torso_controller.move_group.get_end_effector_link()

        # remove any previous objects added to the planning scene
        self.planning_scene.remove_attached_object(eef_link, name='object')
        self.planning_scene.remove_world_object('object')

        co = add_collision_object(objs_resp.object_clouds[obj_idx], self.planning_scene)

        # rospy.sleep(7.0)
        # clear_octomap()
        # rospy.sleep(4.)

        # pclmsg = rospy.wait_for_message('/throttle_filtering_points/filtered_points', PointCloud2, timeout=10)
        # self.test_cloud_pub.publish(pclmsg)
        # rospy.sleep(10.0)0

        # self.arm_torso_controller.sync_reach_safe_joint_space()
        config = dict(planning_time=15., allow_replanning=True)
        self.arm_torso_controller.configure_planner(config)

        rospy.sleep(2.)
        play_motion_action('open')

        print('About to Approach')
        # all_grasps_base = tf_transform('base_footprint', pose_array=grasp_poses).target_pose_array
        # all_grasps_base = self.transform_pose_array(grasp_poses, 'base_footprint')
        self.target_poses_pub.publish(grasp_poses)
        result = self.arm_torso_controller.sync_reach_ee_poses(grasp_poses)
        self.arm_torso_controller.update_planning_scene(add=True)

        if not result:
            print('Approach Failed')
            # home_result = play_motion_action('home')
            # self.arm_torso_controller.sync_reach_safe_joint_space()
            result='failed'
        else:
            print('About to Shift Forward')
            # self.planning_scene.remove_world_object('object')
            # self.arm_torso_controller.update_planning_scene(add=True)

            shift = 0.2
            config['planning_attempts'] = 1
            config['planning_time'] = 5.
            config['num_planning_attempts'] = 5
            self.arm_torso_controller.configure_planner(config)
            for i in range(3):
                print('Shift Try: {}'.format(i + 1))
                result = self.arm_torso_controller.sync_shift_ee(x=shift)
                if result:
                    break
                else:
                    shift -= 0.03

            del config['planning_attempts']
            del config['planning_attempts']
            self.arm_torso_controller.configure_planner(config)

            rospy.sleep(5.)

            if not result:
                print('Shift Forward Failed')
                result='failed'
                # co = self.add_collision_object(objs_resp.object_clouds[obj_ind])
                # home_result = play_motion_action('home')
                # clear_octomap()
                # rospy.sleep(3.)
                self.arm_torso_controller.sync_reach_safe_joint_space()
                self.planning_scene.remove_world_object('object')
            else:
                print('About to Close')
                result = play_motion_action('close')

                rospy.sleep(5.)

                if not result:
                    print('Close Failed')
                    result='failed'
                    # co = self.add_collision_object(objs_resp.object_clouds[obj_ind])
                    # home_result = play_motion_action('home')
                    # self.arm_torso_controller.sync_reach_safe_joint_space()
                    # self.planning_scene.remove_world_object('object')
                else:
                    print('Attaching Object')
                    aco = AttachedCollisionObject()
                    aco.link_name = self.arm_torso_controller.move_group.get_end_effector_link()
                    aco.object = co
                    aco.touch_links = ['gripper_link', 'gripper_left_finger_link', 'gripper_right_finger_link']
                    self.planning_scene.attach_object(aco)

                    rospy.sleep(5.)

                    print('About to Lift')
                    result = self.arm_torso_controller.sync_shift_ee_frame(shift_frame='map', z=0.2)

                    rospy.sleep(5.)

                    if not result:
                        print('Lift Failed')
                        result='failed'
                        # home_result = play_motion_action('home')
                        self.arm_torso_controller.sync_reach_safe_joint_space()
                    else:
                        self.planning_scene.remove_attached_object(eef_link, name='object')
                        self.planning_scene.remove_world_object('object')

                        print('About to Open')
                        result = play_motion_action('open')

                        rospy.sleep(5.)

                        if not result:
                            print('Open Failed')
                            # home_result = play_motion_action('home')
                            self.arm_torso_controller.sync_reach_safe_joint_space()
                        else:
                            print('Open Succeeded')
                            # home_result = play_motion_action('home')
                            self.arm_torso_controller.sync_reach_safe_joint_space()


        self.planning_scene.remove_attached_object(eef_link, name='object')
        self.planning_scene.remove_world_object('object')
        # self.psm_client.update_configuration({'publish_geometry_updates': True })
        
        # shutdown moveit_commander after using it
        moveit_commander.roscpp_shutdown()
        return result

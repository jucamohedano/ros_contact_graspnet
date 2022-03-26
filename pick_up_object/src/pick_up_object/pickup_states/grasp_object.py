#!/usr/bin/python

import rospy
import smach
from pick_up_object.utils import play_motion_action
from moveit_msgs.msg import AttachedCollisionObject

class GraspObject(smach.State):
    
    def __init__(self, arm_torso_controller, gripper_controller, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'collision_obj'],
                             output_keys=['prev', 'collision_obj'])
        self.arm_torso = arm_torso_controller
        self.gripper = gripper_controller
        self.planning_scene = planning_scene

    def execute(self, userdata):
        
        userdata.prev = 'GraspObject'
        collision_obj = userdata.collision_obj
        
        print('About to Close')
        self.arm_torso.update_planning_scene(add=True)
        rospy.sleep(2.)
        result = self.gripper.sync_reach_to(joint1=0.01, joint2=0.01)

        gripper_joints = self.arm_torso.get_joint_values()
        names = self.arm_torso._joints
        print(gripper_joints)
        print(names)
        # result = play_motion_action('close')

        rospy.sleep(1.)

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
            aco.link_name = self.arm_torso.move_group.get_end_effector_link()
            aco.object = collision_obj
            aco.touch_links = ['gripper_link', 'gripper_left_finger_link', 'gripper_right_finger_link']
            self.planning_scene.attach_object(aco)

            rospy.sleep(2.)

            result = 'succeeded'

            
        return result
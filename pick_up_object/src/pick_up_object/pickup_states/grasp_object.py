#!/usr/bin/python

import rospy
import smach
from pick_up_object.utils import play_motion_action
from moveit_msgs.msg import AttachedCollisionObject

class GraspObject(smach.State):
    
    def __init__(self, arm_torso_controller, gripper_controller, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'collision_objs', 'object_index'],
                             output_keys=['prev', 'collision_objs', 'object_index'])
        self.arm_torso = arm_torso_controller
        self.gripper = gripper_controller
        self.planning_scene = planning_scene

    def execute(self, userdata):
        
        userdata.prev = 'GraspObject'
        collision_objs = userdata.collision_objs
        object_index = userdata.object_index
        
        rospy.loginfo('Going to close gripper')

        # self.arm_torso.update_planning_scene(add=True)
        result = self.gripper.sync_reach_to(joint1=0.0, joint2=0.0, wait=220)

        gripper_joints = self.arm_torso.get_joint_values()
        names = self.arm_torso._joints
        # print(gripper_joints)
        # print(names)
        # result = play_motion_action('close')

        rospy.sleep(1.)

        if not result:
            result='failed'
            # co = self.add_collision_object(objs_resp.object_clouds[obj_ind])
            # home_result = play_motion_action('home')
            # self.arm_torso_controller.sync_reach_safe_joint_space()
            # self.planning_scene.remove_world_object('object')
        else:
            rospy.loginfo("Attaching object with index {} to gripper".format(object_index))
            aco = AttachedCollisionObject()
            # aco.link_name = 'gripper_left_finger_link'
            aco.link_name = self.arm_torso.move_group.get_end_effector_link()
            aco.object = collision_objs[0]
            aco.touch_links = ['gripper_link', 'gripper_tool_link', 'gripper_left_finger_link', 'gripper_right_finger_link']
            self.planning_scene.attach_object(aco)

            result = 'succeeded'

            
        return result

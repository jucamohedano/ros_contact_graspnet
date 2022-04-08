#!/usr/bin/python
import rospy
import smach
from pick_up_object.utils import play_motion_action

class PickupRecovery(smach.State):
    
    def __init__(self, arm_torso_controller, gripper_controller, planning_scene):
        smach.State.__init__(self,
                             outcomes=['failed'],
                             input_keys=['prev', 'collision_obj'],
                             output_keys=['prev'])
        self.arm_torso = arm_torso_controller
        self.gripper = gripper_controller
        self.eef_link = arm_torso_controller.move_group.get_end_effector_link()
        self.planning_scene = planning_scene
        

    def execute(self, userdata):
        
        userdata.prev = 'PickupRecovery'
        # collision_obj = userdata.collision_obj
        
        self.planning_scene.remove_attached_object(self.eef_link, name='object')
        self.planning_scene.remove_world_object('object')
        
        self.gripper.sync_reach_to(joint1=0.04, joint2=0.04)
        rospy.sleep(2.)
        self.arm_torso.sync_shift_ee_frame(shift_frame='base_footprint', z=0.2)
        rospy.sleep(2.)
        self.arm_torso.sync_reach_safe_joint_space()
        play_motion_action('open')
        
        return 'failed'
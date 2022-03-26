#!/usr/bin/python
import smach
from pick_up_object.utils import play_motion_action
from moveit_msgs.msg import AttachedCollisionObject

class LiftObject(smach.State):
    
    def __init__(self, gripper_controller, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'collision_obj'],
                             output_keys=['prev', 'collision_obj'])
        self.gripper = gripper_controller
        self.planning_scene = planning_scene
        

    def execute(self, userdata):
        
        userdata.prev = 'LiftObject'
        collision_obj = userdata.collision_obj
        
        print('About to Lift')

        result = self.arm_torso_controller.sync_shift_ee_frame(shift_frame='map', z=0.2)

        rospy.sleep(1.)

        if not result:
            print('Lift Failed')
            result='failed'
            self.arm_torso_controller.sync_reach_safe_joint_space()
        else:
            result = 'succeed'
            
            
            
        return result
#!/usr/bin/python
import smach
import rospy

class LiftObject(smach.State):
    
    def __init__(self, arm_torso_controller, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'collision_obj'],
                             output_keys=['prev', 'collision_obj'])
        self.arm_torso = arm_torso_controller
        self.planning_scene = planning_scene
        

    def execute(self, userdata):
        
        userdata.prev = 'LiftObject'
        # collision_obj = userdata.collision_obj
        
        print('About to Lift')

        result = self.arm_torso.sync_shift_ee_frame(shift_frame='base_footprint', z=0.5)

        rospy.sleep(1.)

        if not result:
            print('Lift Failed')
            result='failed'
            self.arm_torso.sync_reach_safe_joint_space()
        else:
            result = 'succeeded'
            
            
            
        return result
#!/usr/bin/python
import smach
import rospy

class LiftObject(smach.State):
    
    def __init__(self, arm_torso_controller, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'collision_objs'],
                             output_keys=['prev', 'collision_objs'])
        self.arm_torso = arm_torso_controller
        self.planning_scene = planning_scene
        

    def execute(self, userdata):
        
        userdata.prev = 'LiftObject'
        # collision_obj = userdata.collision_obj
        
        rospy.loginfo('Trying to lift object')

        config = dict()
        # config['goal_orien_tol'] = 0.01
        # config['goal_pos_tol'] = 0.01
        # config['goal_joint_tolerance'] = 0.01
        # config['planning_attempts'] = 10
        # config['planning_time'] = 0.5
        # config['num_planning_attempts'] = 10
        # self.arm_torso.configure_planner(config)
        result = self.arm_torso.sync_shift_ee_frame(shift_frame='base_footprint', z=0.25)

        rospy.sleep(1.)
        #del config['goal_orien_tol']
        #del config['goal_pos_tol']
        #del config['goal_joint_tolerance']
        # self.arm_torso.configure_planner(config)

        if not result:
            result='failed'
            self.arm_torso.sync_reach_safe_joint_space()
        else:
            self.planning_scene.remove_world_object()
            result = 'succeeded'
            userdata.prev = 'Pickup' # rename to pickup because we go up to main state machine
            
            
            
        return result

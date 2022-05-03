#!/usr/bin/env python

import rospy
import smach
import random

class GoToTable(smach.State):
    
    def __init__(self, head_controller, base_controller, arm_torso_controller):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev'],
                             output_keys=['prev']
                             )
        self.base = base_controller
        self.head = head_controller
        self.arm_torso = arm_torso_controller
        self.locations = [rospy.get_param('/table_left'), rospy.get_param('/table_right'), rospy.get_param('/tall_table')]

    def execute(self, userdata):
        # randomly choose location 
        loc = self.locations[random.randint(0, 2)]
        rospy.loginfo('Going to table {}'.format(loc['name']))

        userdata.prev = 'GoToTable'
        self.arm_torso.sync_reach_safe_joint_space()
        p = [loc['position']['x'], loc['position']['y'], loc['position']['z']]
        o = [loc['orientation']['x'], loc['orientation']['y'], loc['orientation']['z'], loc['orientation']['w']]
        self.head.sync_reach_to(joint1=0, joint2=0)
        result = self.base.sync_reach_to(position=p, quaternion=o)
        

        if result:
            return 'succeeded'

        return 'failed'

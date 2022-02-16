#!/usr/bin/python
import rospy
import smach
from pick_and_pack_task.utilities import play_motion_action

class Recovery(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'restart', 'restart_object_detection'],
                             input_keys = ['prev'])
        self.restart_count = 0

    def execute(self, userdata):

        if self.restart_count > 3:
            return 'failed'

        if userdata.prev == 'DetectObjects':
            self.restart_count += 1
            return 'restart'

        if userdata.prev == 'GenerateGrasps':
            self.restart_count += 1
            return 'restart'

        if userdata.prev == 'Pickup':
            self.restart_count += 1
            return 'restart_object_detection'

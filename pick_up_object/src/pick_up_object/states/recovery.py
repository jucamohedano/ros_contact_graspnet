#!/usr/bin/python

import smach

class Recovery(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['failed', 'restart', 'change_grasps'],
                             input_keys = ['prev'], 
                             output_keys = ['prev'])
        self.restart_count = 0

    def execute(self, userdata):
    
        if self.restart_count == 20:
            userdata.prev = 'Recovery'
            return 'failed'

        if userdata.prev == 'DetectObjects':
            self.restart_count += 1
            userdata.prev = 'Recovery'
            return 'restart'

        if userdata.prev == 'GenerateGrasps':
            self.restart_count += 1
            userdata.prev = 'Recovery'
            return 'restart'

        if userdata.prev == 'Pickup':
            self.restart_count += 1
            userdata.prev = 'Recovery'
            return 'change_grasps'

        if userdata.prev == 'DecideGraspsAndObjs':
            self.restart_count += 1
            userdata.prev = 'Recovery'
            return 'restart'

    

#!/usr/bin/python
import rospy
import smach
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from pick_and_pack_task.utilities import detect_objs


class DetectObjects(smach.State):
    
    def __init__(self, head_controller, torso_controller):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'looping', 'failed'],
                             output_keys=['objs_resp', 'prev'])
        self.head_controller = head_controller
        self.torso_controller = torso_controller
        self.retry_attempts = 3
        self.try_num = 0

    def execute(self, userdata):
        userdata.prev = 'DetectObjects'

        # lift torso to get a good top view
        # self.torso_controller.sync_reach_to(joint1=0.35) 

        # look down
        result = self.head_controller.sync_reach_to(joint1=0, joint2=-0.98)
        # TODO: handle error

        objs_resp = detect_objs()
        self.try_num += 1

        # check that we have objects. Approach has to be changed because we are not using mask rcnn
        if len(objs_resp.object_clouds) > 0:
            userdata.objs_resp = objs_resp
            return 'succeeded'
        elif self.try_num <= self.retry_attempts:
            return 'looping'
        else:
            return 'failed'

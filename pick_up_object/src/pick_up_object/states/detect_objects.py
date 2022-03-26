#!/usr/bin/python
import rospy
import smach
import numpy as np
from pick_up_object.utils import detect_objs, play_motion_action

home_pose_joint_val = np.array([0.2, -1.3387, -0.2, 1.9385, -1.57, 1.3698])

class DetectObjects(smach.State):
    
    def __init__(self, head_controller, torso_controller, arm_torso_controller):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'looping', 'failed'],
                             input_keys=['prev'],
                             output_keys=['objs_resp', 'prev'])
        self.head_controller = head_controller
        self.torso_controller = torso_controller
        self.arm_torso_controller = arm_torso_controller
        self.retry_attempts = 3
        self.try_num = 0

    def execute(self, userdata):
        
        # avoid resetting home position if you are already in it
        # curr_joint_val might be empty if there are synchronization clock problems
        # rospy.sleep(2.)
        
        play_home_motion = False
        try:
            curr_joints_val = self.arm_torso_controller._move_group.get_current_joint_values()
            if curr_joints_val[-1] > 0.32:
                self.arm_torso_controller.sync_reach_safe_joint_space()
                play_home_motion = False
        except rospy.ROSException:
            play_home_motion = True
        # curr_joints_val = curr_joints_val[1:-1] # dont want torso
        # curr_joints_val = np.array([round(j, 4) for j in curr_joints_val])
        # print('curr_joints_val', curr_joints_val)
        # print('home_pose_joint_val', home_pose_joint_val)

        # if not np.allclose(curr_joints_val, home_pose_joint_val, rtol=1e-03, atol=1e-05):
        #     print('not close enough')
        if play_home_motion:
            play_motion_action('home')
        # lift torso to get a good top view
        self.torso_controller.sync_reach_to(joint1=0.35) 
        
        userdata.prev = 'DetectObjects'

        # look down
        self.head_controller.sync_reach_to(joint1=0, joint2=-0.98)
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

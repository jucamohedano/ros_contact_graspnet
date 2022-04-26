#!/usr/bin/python
import rospy
import smach
import numpy as np
from sensor_msgs.msg import PointCloud2
from pick_up_object.utils import detect_objs, play_motion_action, detect_clusters

home_pose_joint_val = np.array([0.2, -1.3387, -0.2, 1.9385, -1.57, 1.3698])


class DetectObjects(smach.State):
    
    def __init__(self, head_controller, torso_controller, arm_torso_controller, detector='rcnn'):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'looping', 'failed'],
                             input_keys=['prev'],
                             output_keys=['objs_resp', 'prev'])
        self.head_controller = head_controller
        self.torso_controller = torso_controller
        self.arm_torso_controller = arm_torso_controller
        self.retry_attempts = 2
        self.try_num = 0
        self.detector = self._detectRCNN if detector == 'rcnn' else self._detectPCL

    def execute(self, userdata):
        self.arm_torso_controller._scene.clear()
        userdata.prev = 'DetectObjects'
        self.arm_torso_controller.sync_reach_safe_joint_space()
        # lift torso to get a good top view
        self.torso_controller.sync_reach_to(joint1=0.35)
        # look down
        self.head_controller.sync_reach_to(joint1=0, joint2=-0.98)
        return self.detector(userdata)
        
    def _detectPCL(self, userdata):
        pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2, timeout=10)
        objs_resp = detect_clusters(pcl)
        print('Euclidian clusters detected are {}'.format(len(objs_resp.clusters)))
        self.try_num += 1

        # check that we have objects. Approach has to be changed because we are not using mask rcnn
        if len(objs_resp.clusters) > 0:
            userdata.objs_resp = objs_resp
            return 'succeeded'
        elif self.try_num <= self.retry_attempts:
            return 'looping'
        else:
            return 'failed'

    def _detectRCNN(self, userdata):
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

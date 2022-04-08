#!/usr/bin/python

import rospy
import smach
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
from pick_up_object.utils import generate_grasps
# import rosnode
# from subprocess import call
import os

class GenerateGrasps(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'looping', 'failed'],
                             input_keys=['prev', 'objs_resp'],
                             output_keys=['prev', 'grasps_resp'])
        self.retry_attempts = 3
        self.try_num = 0


    def execute(self, userdata):
        
        userdata.prev = 'GenerateGrasps'

        # grasps_resp = self.predict_full_scene()
        grasps_resp = self.generate_grasps()
        
        self.try_num += 1

        pose_count = np.sum([len(grasp_poses.poses) for grasp_poses in grasps_resp.all_grasp_poses])
        if pose_count > 0:
            userdata.grasps_resp = grasps_resp
            return 'succeeded'
        elif self.try_num < self.retry_attempts:
            return 'looping'
        else:
            return 'failed'


    def predict_full_scene(self):
        
        pcl_msg = rospy.wait_for_message('/throttle_filtering_points/filtered_points', PointCloud2)

        pcl = ros_numpy.numpify(pcl_msg)
        pcl = np.concatenate( (pcl['x'].reshape(-1,1), pcl['y'].reshape(-1,1), pcl['z'].reshape(-1,1)), axis=1)
        
        grasps_resp = generate_grasps(pcl_msg, {})
        return grasps_resp
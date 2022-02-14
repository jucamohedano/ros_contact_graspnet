#!/usr/bin/python

import rospy
import smach
import numpy as np

class GenerateGrasps(smach.State):
    
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             output_keys=['grasps_resp'])

    def execute(self, userdata):

        full_pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        full_pcl = ros_numpy.numpify(full_pcl)
        full_pcl = np.concatenate( (full_pcl['x'].reshape(-1,1), full_pcl['y'].reshape(-1,1), full_pcl['z'].reshape(-1,1)), axis=1)

        pose_count = np.sum([len(grasp_poses.poses) for grasp_poses in grasps_resp.all_grasp_poses])
       
        if pose_count > 0:
            userdata.grasps_resp = grasps_resp
            return 'succeeded'
        else:
            return 'failed'

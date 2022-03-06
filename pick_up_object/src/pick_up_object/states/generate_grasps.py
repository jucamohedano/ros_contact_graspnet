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
                             input_keys=['objs_resp', 'prev'],
                             output_keys=['prev', 'grasps_resp', 'objs_resp'])
        self.retry_attempts = 3
        self.try_num = 0


    def execute(self, userdata):
        # rosnode.kill_nodes('maskRCNN_detection')
        # call(["rosnode", "kill", "maskRCNN_detection"], shell=True)
        # os.system("rosnode kill maskRCNN_detection")

        userdata.prev = 'GenerateGrasps'

        objs_resp = userdata.objs_resp 

        grasps_resp = generate_grasps(objs_resp.full_pcl, objs_resp.object_clouds)
        self.try_num += 1

        pose_count = np.sum([len(grasp_poses.poses) for grasp_poses in grasps_resp.all_grasp_poses])

        # f_pose_count = np.sum([len(grasp_poses.poses) for grasp_poses in grasps_resp.filtered_grasp_poses])
        # print('pose_count: {}, f_pose_count: {}'.format(pose_count, f_pose_count))
        if pose_count > 0:
            userdata.grasps_resp = grasps_resp
            return 'succeeded'
        elif self.try_num < self.retry_attempts:
            return 'looping'
        else:
            return 'failed'


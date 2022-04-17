#!/usr/bin/python

import os
import rospy
import smach
import numpy as np
import ros_numpy

from sensor_msgs.msg import PointCloud2
from pick_up_object.utils import generate_grasps, tf_transform, get_poses_around_obj, generate_antipodal_grasps
from pick_up_object.srv import AntipodalGraspRequest
from pcl_manipulation.srv import EuclidianResponse
# import rosnode
# from subprocess import call


class GenerateGrasps(smach.State):
    def __init__(self, arm_torso_controller, grasping_method='contact_graspnet'):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'looping', 'failed'],
                             input_keys=['prev', 'objs_resp'],
                             output_keys=['prev', 'grasps_resp'])
        self.retry_attempts = 3
        self.try_num = 0
        self.arm_torso = arm_torso_controller
        self.grasping_method = self._contactGraspnet if grasping_method == 'contact_graspnet' else self._geometricGrasping

    def execute(self, userdata):
        userdata.prev = 'GenerateGrasps'
        
        grasps_resp = self.grasping_method(userdata)
        self.try_num += 1
        
        if grasps_resp:
            userdata.grasps_resp = grasps_resp
            return 'succeeded'
        elif self.try_num < self.retry_attempts:
            return 'looping'
        else:
            return 'failed'

    def _contactGraspnet(self, userdata):
        objs_resp = userdata.objs_resp
        pcl_msg = objs_resp.full_pcl
        objects = objs_resp.object_clouds
   
        grasps_resp = generate_grasps(pcl_msg, objects)

        pose_count = np.sum([len(grasp_poses.poses) for grasp_poses in grasps_resp.all_grasp_poses])
        if pose_count > 0:
            return grasps_resp.all_grasp_poses
        else:
            return False

    def _geometricGrasping(self, userdata):
        # get head camera position
        try:
            cam_pose = self.tfBuffer.lookup_transform('base_footprint', 'xtion_depth_optical_frame', rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)

        objs_resp = userdata.objs_resp
        obj = objs_resp.object_clouds[0]
        obj = tf_transform('base_footprint', pointcloud=obj).target_pointcloud
        poses_around_obj = get_poses_around_obj(obj)
        print(obj.header.frame_id)
        # obj = tf_transform('base_footprint', pointcloud=obj).target_pointcloud
        # move to those poses to get the pointclouds
        #self.arm_torso.configure_planner()
        pcd = []
        pcd.append(obj) # append the pointcloud of the object itself since we have it already
        for i, pose in enumerate(poses_around_obj.poses):
            self.arm_torso.sync_reach_ee_pose((pose.position.x, pose.position.y, pose.position.z), 
                                              (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            pcl = rospy.wait_for_message('/camera/depth/color/points', PointCloud2, timeout=10.)
            pcl = tf_transform('base_footprint', pointcloud=pcl).target_pointcloud
            pcd.append(pcl)

        pcl = ros_numpy.numpify(obj)
        pcl_np = np.concatenate( (pcl['x'].reshape(-1,1), pcl['y'].reshape(-1,1), pcl['z'].reshape(-1,1)), axis=1)
        pcl_min = np.nanmin(pcl_np, axis=0)
        pcl_max = np.nanmax(pcl_np, axis=0)

        # update poses with head camera pose for normal orientation in the antipodal server
        cam_pose = cam_pose.transform
        poses_around_obj.poses.insert(0, Pose(
                                            Point(cam_pose.translation.x, 
                                                  cam_pose.translation.y, 
                                                  cam_pose.translation.z),
                                            Quaternion(cam_pose.rotation.x, 
                                                       cam_pose.rotation.y, 
                                                       cam_pose.rotation.z, 
                                                       cam_pose.rotation.w))
                                            )
        req = AntipodalGraspRequest(pcd, 
                                    poses_around_obj, 
                                    pcl_min[0], 
                                    pcl_min[1], 
                                    pcl_min[2], 
                                    pcl_max[0], 
                                    pcl_max[1], 
                                    pcl_max[2]
                                    )

        antipodal_grasps = generate_antipodal_grasps(req)

        if len(antipodal.poses) > 0:
            return antipodal_grasps.all_grasp_poses
        else:
            return False

#!/usr/bin/env python

import rospy
# import open3d as o3d
import numpy as np

from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from geometric_grasp.srv import PointcloudReconstruction, PointcloudReconstructionResponse
# from geometric_grasp.utils import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d 

def reconstruct(msg):
    # pcl_response = PointcloudReconstructionResponse()
    
    # o3d_cloud = convertCloudFromRosToOpen3d(msg)
    # mean, _ = o3d.compute_mean_and_covariance(o3d_cloud)
    compute_poses_around_pose(mean)


def compute_poses_around_pose(pose):
    """
    Arguments
        pose {Pose} - mean pose of the object
    Return
        set of poses around the object
    """
    # set of hard coded positions around the object
    pose_array = PoseArray()
    pose_array.header.frame_id = pose.header.frame_id
    pose_array.header.stamp = rospy.Time.now()

    in_front_pose = Pose()
    in_front_pose.position = Point(pose[0]-0.02, pose[1], pose[2]+0.03)
    
    # vectors to mean
    new_orientation = np.array(pose[0]-0.02, pose[1], pose[2]+0.03) - np.array(pose[0], pose[1], pose[2])
    pose = pose_from_vector3D(new_orientation)
    in_front_pose.orientation = pose.orientation
    # right_front = [pose[0], pose[1]+0.02, pose[2]+0.03]
    # left_back = [pose[0], pose[1]-0.02, pose[2]+0.03]
    pub.publish(in_front_pose)


def pose_from_vector3D(vector):
    #http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    pose= Pose()
    pose.position.x = vector[0]
    pose.position.y = vector[1]
    pose.position.z = vector[2] 
    #calculating the half-way vector.
    u = [1,0,0]
    norm = np.linalg.norm(vector[3:])
    v = np.asarray(vector[3:])/norm 
    if (np.array_equal(u, v)):
        pose.orientation.w = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
    elif (np.array_equal(u, np.negative(v))):
        pose.orientation.w = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 1
    else:
        half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
        pose.orientation.w = np.dot(u, half)
        temp = np.cross(u, half)
        pose.orientation.x = temp[0]
        pose.orientation.y = temp[1]
        pose.orientation.z = temp[2]
    norm = np.math.sqrt(pose.orientation.x*pose.orientation.x + pose.orientation.y*pose.orientation.y + 
        pose.orientation.z*pose.orientation.z + pose.orientation.w*pose.orientation.w)
    if norm == 0:
        norm = 1
    pose.orientation.x /= norm
    pose.orientation.y /= norm
    pose.orientation.z /= norm
    pose.orientation.w /= norm
    return pose

if __name__ == '__main__':
    
    rospy.init_node('pointcloud_reconstruction')
    s = rospy.Service('pointcloud_reconstruction', PointcloudReconstruction, reconstruct)
    pub = rospy.Publisher('/debugger_reconstruct', Pose, queue_size=1)

    print('Ready to stich pointcloud!')
    rospy.spin()

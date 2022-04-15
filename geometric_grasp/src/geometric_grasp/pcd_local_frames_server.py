#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
import copy
import ros_numpy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, PoseArray
from contact_graspnet.utils import tf_transform

from geometric_grasp.srv import PcdLocalFrames, PcdLocalFramesResponse
from geometric_grasp.utils import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d, o3dpc_to_rospc
from sensor_msgs.msg import PointCloud2

def local_frames(msg):
    frame_id = msg.full_reconstructed_pcd.header.frame_id
    # msg_pcl = tf_transform('gripper_grasping_frame', pointcloud=msg.full_reconstructed_pcd).target_pointcloud
    cloud = convertCloudFromRosToOpen3d(msg.full_reconstructed_pcd)
    down_sampled_pcd = cloud.voxel_down_sample(voxel_size=0.005)
    pub.publish(o3dpc_to_rospc(down_sampled_pcd, frame_id=frame_id, stamp=rospy.Time.now()))

    cloud_npy = np.asarray(copy.deepcopy(down_sampled_pcd.points))
    indices = np.random.choice(range(cloud_npy.shape[0]), size=np.min([500, cloud_npy.shape[0]]), replace=False)
    kdtree = o3d.geometry.KDTreeFlann(down_sampled_pcd)

    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = frame_id
    for i in indices:
        query = cloud_npy[i,:]
        (num, indices, distances) = kdtree.search_hybrid_vector_3d(query=query, radius=0.1, max_nn=40)

        cluster = cloud_npy[indices]
        cluster = create_open3d_point_cloud(cluster)
        mean, cov = cluster.compute_mean_and_covariance()
        w, V = np.linalg.eig(cov)
        Rot = np.fliplr(V)
        # Handle improper rotations
        Rot = np.diag([1, 1, np.linalg.det(Rot)]) @ Rot
        # Flip normals
        if Rot[0,2] < 0:
            Rot = -Rot
        r = R.from_matrix(Rot)

        quat = r.as_quat()
        pose = Pose()
        pose.position.x = query[0]
        pose.position.y = query[1]
        pose.position.z = query[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose_array.poses.append(pose)

    local_basis_pub.publish(pose_array)
    resp = PcdLocalFramesResponse(pose_array)
    return resp

def create_open3d_point_cloud(point_cloud):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    return pcd


if __name__ == '__main__':
    
    rospy.init_node('pcd_local_frames')
    s = rospy.Service('pcd_local_frames', PcdLocalFrames, local_frames)
    pub = rospy.Publisher('/downsampled_cloud', PointCloud2, queue_size=1, latch=True)
    local_basis_pub = rospy.Publisher('/all_local_basis', PoseArray, queue_size=1, latch=True)

    print('Ready to compute local basis!')
    rospy.spin()

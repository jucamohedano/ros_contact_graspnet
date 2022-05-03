#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np

from geometric_grasp.srv import PointcloudReconstruction, PointcloudReconstructionResponse
from geometric_grasp.utils import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d, o3dpc_to_rospc
from sensor_msgs.msg import PointCloud2

def merge_pointclouds(msg):
    resp = PointcloudReconstructionResponse()
    frame_id = msg.object_clouds[0].header.frame_id
    object_clouds_o3d = [convertCloudFromRosToOpen3d(cloud) for cloud in msg.object_clouds]
    
    merged_pcd = o3d.geometry.PointCloud()
    for i in range(len(object_clouds_o3d)):
        merged_pcd += object_clouds_o3d[i]
    # downsample
    # merged_pcd_down = merged_pcd.voxel_down_sample(voxel_size=0.01)
    # merged_pcd_down_cropped = merged_pcd.crop(
            # o3d.geometry.AxisAlignedBoundingBox(min_bound=[-0.5, -0.3, -0.8],
            #                                     max_bound=[0.5, 0.3, 0.8]))
    offset = 0.05
    # merged_pcd_down_cropped = merged_pcd.crop(
    #                             o3d.geometry.AxisAlignedBoundingBox(
    #                                                 min_bound=np.array([msg.min_x-offset, msg.min_y-offset, msg.min_z-offset]),
    #                                                 max_bound=np.array([msg.max_x+offset, msg.max_y+offset, msg.max_z+offset]))
    #                                                 )
    pcd_mask = convertCloudFromRosToOpen3d(msg.object_clouds[0]) # cloud of the object itself
    

    bb = pcd_mask.get_oriented_bounding_box(robust=True)
    merged_pcd_cropped = merged_pcd.crop(bb)

    # plane_model, inliers = merged_pcd_cropped.segment_plane(distance_threshold=0.01,
    #                                     ransac_n=3,
    #                                     num_iterations=1000)
    # inlier_cloud = merged_pcd_cropped.select_by_index(inliers)
    # outlier_cloud = merged_pcd_cropped.select_by_index(inliers, invert=True)


    # outlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    #         radius=0.1, max_nn=30))

    # ros_pcd = o3dpc_to_rospc(merged_pcd_cropped, frame_id='base_footprint', stamp=rospy.Time.now())
    outlier_cloud = o3dpc_to_rospc(merged_pcd_cropped, frame_id=frame_id, stamp=rospy.Time.now())
    resp.full_reconstructed_pcd = outlier_cloud
    pub.publish(outlier_cloud)
    return resp




if __name__ == '__main__':
    
    rospy.init_node('reconstruct_pointcloud')
    s = rospy.Service('reconstruct_pointcloud', PointcloudReconstruction, merge_pointclouds)
    pub = rospy.Publisher('/reconstructed_pcd', PointCloud2, queue_size=1, latch=True)

    print('Ready to stich pointcloud!')
    rospy.spin()



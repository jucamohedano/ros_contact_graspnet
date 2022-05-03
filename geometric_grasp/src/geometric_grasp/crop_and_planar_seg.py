#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np

from geometric_grasp.srv import PointcloudSceneSegmentation, PointcloudSceneSegmentationResponse
from geometric_grasp.utils import convertCloudFromRosToOpen3d, o3dpc_to_rospc
from sensor_msgs.msg import PointCloud2

def callback(msg):
    frame_id = msg.object_clouds[0].header.frame_id
    
    # convert each cloud to open3d from ROS
    # object_clouds_o3d = [(convertCloudFromRosToOpen3d(cloud)).voxel_down_sample(voxel_size=0.005) for cloud in msg.object_clouds]
    object_clouds_o3d = [convertCloudFromRosToOpen3d(cloud) for cloud in msg.object_clouds]

    # base_footprint AxisAlignedBoundingBox to crop point cloud noise
    pcd_cropped = object_clouds_o3d[0].crop(o3d.geometry.AxisAlignedBoundingBox(
                                np.array([0.3, -0.3, 0.]),
                                np.array([1., 0.3, 1.5])
                                ))

    # segment table plane
    _, inliers = pcd_cropped.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
    seg_cloud = pcd_cropped.select_by_index(inliers, invert=True)

    ros_pcd = o3dpc_to_rospc(seg_cloud, frame_id='base_footprint', stamp=rospy.Time.now(0))
    resp = PointcloudSceneSegmentationResponse(ros_pcd)
    return resp



if __name__ == '__main__':
    
    rospy.init_node('crop_and_planar_seg')
    s = rospy.Service('crop_and_planar_seg', PointcloudSceneSegmentation, callback)
    pub = rospy.Publisher('/new_pcd_scene', PointCloud2, queue_size=1, latch=True)

    print('Ready to crop and segment pointcloud!')
    rospy.spin()



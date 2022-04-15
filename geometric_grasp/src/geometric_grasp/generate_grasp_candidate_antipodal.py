#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from geometric_grasp.srv import AntipodalGrasp, AntipodalGraspResponse
from geometric_grasp.utils import convertCloudFromRosToOpen3d, o3dpc_to_rospc
from sensor_msgs.msg import PointCloud2

def generate_candidates(msg):
    frame_id = msg.object_clouds[0].header.frame_id
    
    # convert each cloud to open3d from ROS
    # object_clouds_o3d = [(convertCloudFromRosToOpen3d(cloud)).voxel_down_sample(voxel_size=0.005) for cloud in msg.object_clouds]
    object_clouds_o3d = [convertCloudFromRosToOpen3d(cloud) for cloud in msg.object_clouds]

    # compute normals for each cloud now that we now the camera position
    for i, cloud in enumerate(object_clouds_o3d):
        cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))
        cam_loc = msg.target_poses.poses[i].position
        cloud.orient_normals_towards_camera_location([cam_loc.x, cam_loc.y, cam_loc.z])
        
    # merge the point clouds
    merged_pcd = o3d.geometry.PointCloud()
    for i in range(len(object_clouds_o3d)):
        merged_pcd += object_clouds_o3d[i]

    # base_footprint AxisAlignedBoundingBox to crop point cloud noise
    merged_pcd_cropped = merged_pcd.crop(o3d.geometry.AxisAlignedBoundingBox(
                                np.array([0.3, -0.3, 0.]),
                                np.array([1., 0.3, 1.5])
                                ))
    # segment table plane
    _, inliers = merged_pcd_cropped.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
    seg_cloud = merged_pcd_cropped.select_by_index(inliers, invert=True)

    # OrientedAlignBoundingBox
    oriented_bb = seg_cloud.get_oriented_bounding_box(robust=True)  
    new_cloud = seg_cloud.crop(oriented_bb)

    # get the 8 points that form the bounding box
    bb_pts = oriented_bb.get_box_points()
    bb_pts = np.asarray(bb_pts)
    
    # publish the new merged-segmented cloud
    cloud = o3dpc_to_rospc(new_cloud, frame_id=frame_id, stamp=rospy.Time.now())
    rec_pub.publish(cloud)
    
    # choose a random point from the pointcloud
    rng = np.random.default_rng()
    index = rng.integers(0,len(new_cloud.points)-1, size=200)

    pose_array = PoseArray()
    pose_array.header.frame_id = frame_id
    pose_array.header.stamp = rospy.Time.now()

    for i in index:
        # random point on object's surface
        p_WS = np.asarray(new_cloud.points[i])
        # point = PointStamped()
        # point.header = Header(200, rospy.Time.now(), frame_id)
        # point.point = Point(p_WS[0], p_WS[1], p_WS[2])
        # random_pcd_point.publish(point)
        n_WS = np.asarray(new_cloud.normals[i])

        assert np.isclose(np.linalg.norm(n_WS), 1.0)

        Gy = n_WS # gripper y axis aligns with normal
        # make orthonormal x axis, aligned with world down
        x = np.array([0.0, 0.0, -1.0])
        if np.abs(np.dot(x, Gy)) < 1e-6:
            # normal was pointing straight down.  reject this sample.
            return None

        
        Gx = x - np.dot(x,Gy)*Gy
        Gz = np.cross(Gy, Gx)
        X_G = R.from_matrix(np.vstack((Gx, Gy, Gz)).T) # candidate grasp

        
        # R_wg = R.from_matrix(np.vstack((Gy, Gx, Gz)).T)
        # quat = R_wg.as_quat()
        # pose = PoseStamped(
        #       Header(20, rospy.Time.now(), 'base_footprint'),
        #       Pose(
        #             Point( p_WS[0], p_WS[1], p_WS[2] ),
        #             Quaternion( quat[0], quat[1], quat[2], quat[3]) )
        #           )
        # obj_basis_pub.publish(pose)
        cost(X_G, p_WS, pose_array, new_cloud)

        # Try orientations from the center out
        min_roll=-np.pi/2.0
        max_roll=np.pi/2.0
        alpha = np.array([0.5, 0.65, 0.35, 0.8, 0.2, 1.0, 0.0])
        for theta in (min_roll + (max_roll - min_roll)*alpha):
            # Rotate the gripper by a random rotation (around the normal).
            theta_r = R.from_euler('y', theta, degrees=False)
            X_G2 = X_G.as_matrix().dot(theta_r.as_matrix())
            cost(R.from_matrix(X_G2), p_WS, pose_array, new_cloud)

            # rotate around x
            theta_r = R.from_euler('x', theta, degrees=False)
            X_G3 = X_G.as_matrix().dot(theta_r.as_matrix())
            cost(R.from_matrix(X_G3), p_WS, pose_array, new_cloud)

    print('generated {} antipodal grasps!'.format(len(pose_array.poses)))
    antipodal_pub.publish(pose_array)

    resp = AntipodalGraspResponse(pose_array)
    return resp


def cost(X_G, p_WS, pose_array, cloud):
    """
    Arguments:
        X_G {3x3 Rotation Matrix} - Gx,Gy,Gz grasping frame of candidate grasp (World coords)
        p_WS                      - contact point in world coords
        cloud                     - object point cloud
    """


    # Transform cloud into gripper frame
    # Crop to a region inside of the finger box.
    crop_min = [0., -0.041, -0.03, 1.]
    crop_max = [0.13625, 0.041, 0.03, 1.]

    X_G = X_G.as_matrix()
    rigid_transform = np.vstack((np.hstack((X_G, p_WS.reshape(3,1))), [0, 0, 0 ,1]))
    

    quat = R.from_matrix(X_G).as_quat()
    grasp = rigid_transform.dot(np.array([-0.05, 0., 0, 1]))
    grasp_box = rigid_transform.dot(np.array([0, 0., 0, 1]))    
    
    # update rigid transform 4x4
    rigid_transform = np.vstack((np.hstack((X_G, grasp[:3].reshape(3,1))), [0, 0, 0 ,1]))
    
    # DEBUGGING - max and min points of bounding box
    # min_point = PointStamped()
    # min_point.header = Header(109, rospy.Time.now(), 'base_footprint')
    # min_point.point = Point(min_p[0], min_p[1], min_p[2])
    # rotated_random_pcd_point.publish(min_point)
    # max_point = PointStamped()
    # max_point.header = Header(99, rospy.Time.now(), 'base_footprint')
    # max_point.point = Point(max_p[0], max_p[1], max_p[2])
    # random_pcd_point.publish(max_point)

    
    X_GW = np.linalg.inv(rigid_transform)
    points = np.asarray(cloud.points)
    ones = np.ones(points.shape[0])
    cloud_pts = np.column_stack((points, ones))
    p_GC = X_GW.dot(cloud_pts.T).T
    
    # compute possible collisions with gripper fingers
    indices_overflow_y_min = np.all((crop_min[0] <= p_GC[:,0], p_GC[:,0] <= crop_max[0],
                                 crop_min[1] > p_GC[:,1],
                                 crop_min[2] <= p_GC[:,2], p_GC[:,2] <= crop_max[2]),
                                 axis=0)
    indices_overflow_y_max = np.all((crop_min[0] <= p_GC[:,0], p_GC[:,0] <= crop_max[0],
                                 p_GC[:,1] > crop_max[1],
                                 crop_min[2] <= p_GC[:,2], p_GC[:,2] <= crop_max[2]),
                                 axis=0)
    
    p_G = np.concatenate((p_GC[indices_overflow_y_min], p_GC[indices_overflow_y_max]), axis=0)
    # pointcloud = create_open3d_point_cloud(p_G[:,:3])
    # ros_pcd = o3dpc_to_rospc(pointcloud, frame_id='base_footprint', stamp=rospy.Time.now())
    # pose = PoseStamped(
    #             Header(20, rospy.Time.now(), 'base_footprint'),
    #             Pose(
    #                 Point( grasp[0], grasp[1], grasp[2] ),
    #                 Quaternion( quat[0], quat[1], quat[2], quat[3]) ))
    # obj_basis_pub.publish(pose)
    # visualise_grasp_box(
    #     Pose( Point( grasp_box[0], grasp_box[1], grasp_box[2] ),
    #           Quaternion(quat[0], quat[1], quat[2], quat[3])),
    #     id=44, frame_id='base_footprint')
    # rec_pub_one.publish(ros_pcd)


    if not np.size(p_G):
        pose_array.poses.append(Pose(Point(grasp[0], grasp[1], grasp[2]), Quaternion(quat[0], quat[1], quat[2], quat[3])))
        
        # VISUALIZATION FOR DEBUGGING
        # pose = PoseStamped(
        #         Header(20, rospy.Time.now(), 'base_footprint'),
        #         Pose(
        #             Point( p[0], p[1], p[2] ),
        #             Quaternion( quat[0], quat[1], quat[2], quat[3]) )
        #             )
        # obj_basis_pub.publish(pose)
        # visualise_grasp_box(
        # Pose( Point( p_box[0], p_box[1], p_box[2] ),
        #         Quaternion(quat[0], quat[1], quat[2], quat[3]) 
        #     ),
        # id=44, frame_id='base_footprint')
        # rospy.sleep(4.)



def create_open3d_point_cloud(point_cloud):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    return pcd

def visualise_grasp_box(pose, id, frame_id, g=1.0):
    vis_pose = Marker()
    vis_pose.pose = pose
    vis_pose.header.frame_id = frame_id
    vis_pose.id = id
    vis_pose.type = Marker.CUBE
    vis_pose.action = Marker.ADD
    vis_pose.scale.x = 0.13625
    vis_pose.scale.y = 0.08
    vis_pose.scale.z = 0.03
    vis_pose.color.r = 1.0
    vis_pose.color.a = 1.0
    vis_pose.lifetime = rospy.Duration()
    one_marker_pub.publish(vis_pose)

if __name__ == '__main__':
    
    rospy.init_node('generate_grasp_candidate_antipodal')
    s = rospy.Service('generate_grasp_candidate_antipodal', AntipodalGrasp, generate_candidates)
    antipodal_pub = rospy.Publisher('/antipodals', PoseArray, queue_size=1, latch=True)
    # rec_pub_one = rospy.Publisher('/reconstructed_pcd_one', PointCloud2, queue_size=1, latch=True)
    rec_pub = rospy.Publisher('/reconstructed_pcd', PointCloud2, queue_size=1, latch=True)
    # random_pcd_point = rospy.Publisher('/random_pt', PointStamped, queue_size=1, latch=True)
    # rotated_random_pcd_point = rospy.Publisher('/rotated_random_pt', PointStamped, queue_size=1, latch=True)
    # rotated_pcd = rospy.Publisher('/rotated_pcd', PointCloud2, queue_size=1, latch=True)
    # rand_basis_pub = rospy.Publisher('/random_point_basis', PoseArray, queue_size=1, latch=True)
    # obj_basis_pub = rospy.Publisher('/obj_basis_pub', PoseStamped, queue_size=1, latch=True)
    # marker_pub = rospy.Publisher('/bb_corners', MarkerArray, queue_size = 8)
    # one_marker_pub = rospy.Publisher('/box', Marker, queue_size = 1, latch=True)
    # local_basis_pub = rospy.Publisher('/all_local_basis', PoseArray, queue_size=1, latch=True)

    print('Ready to generate_grasp_candidate_antipodal!')
    rospy.spin()

#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy
import time

from geometric_grasp.srv import AntipodalGrasp, AntipodalGraspResponse
from geometric_grasp.utils import convertCloudFromRosToOpen3d, o3dpc_to_rospc
from sensor_msgs.msg import PointCloud2
from contact_graspnet.utils.transformations import * # python3 can't import tf.transformations
# from contact_graspnet.utils import tf_transform
import math


def generate_candidates(msg):
    startTime = time.time()
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

    publish_cloud(merged_pcd)

    # base_footprint AxisAlignedBoundingBox to crop point cloud noise
    merged_pcd_cropped = merged_pcd.crop(o3d.geometry.AxisAlignedBoundingBox(
                                np.array([msg.min_x-0.03, msg.min_y-0.03, msg.min_z+0.0]),
                                np.array([msg.max_x+0.03, msg.max_y+0.03, msg.max_z+0.06])
                                ))
    # publish_cloud(merged_pcd_cropped)
    print(merged_pcd_cropped.points)
    # segment table plane
    if len(merged_pcd_cropped.points) < 100:
        return AntipodalGraspResponse([])
    _, inliers = merged_pcd_cropped.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=500)
    seg_cloud = merged_pcd_cropped.select_by_index(inliers, invert=True)
    if (len(seg_cloud.points) == 0):
        seg_cloud = merged_pcd_cropped

    # OrientedAlignBoundingBox
    print(seg_cloud.points)
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
    print(len(new_cloud.points)-1)
    index = rng.integers(0,len(new_cloud.points)-1, size=200)

    pose_array = PoseArray()
    pose_array.header.frame_id = frame_id

    debug_pose_array = PoseArray()
    debug_pose_array.header.frame_id = frame_id
    

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

        
        R_wg = R.from_matrix(np.vstack((Gx, Gy, Gz)).T)
        quat = R_wg.as_quat()
        pose = PoseStamped(
              Header(20, rospy.Time.now(), 'base_footprint'),
              Pose(
                    Point( p_WS[0], p_WS[1], p_WS[2] ),
                    Quaternion( quat[0], quat[1], quat[2], quat[3]) )
                  )
        obj_basis_pub.publish(pose)
        # cost(X_G, p_WS, new_cloud)

        # Try orientations from the center out
        min_roll=-np.pi/3.0
        max_roll=np.pi/3.0
        alpha = np.array([0.5, 0.65, 0.35, 0.8, 0.2, 1.0, 0.0, 0.7, 0.9, 0.4, 0.15])
        for theta in (min_roll + (max_roll - min_roll)*alpha):
            # Rotate the gripper by a random rotation (around the normal).
            theta_r = R.from_euler('y', theta, degrees=False)
            X_G2 = X_G.as_matrix().dot(theta_r.as_matrix())
            X_G2_pose_array, debuggin_pose_array = cost(R.from_matrix(X_G2), p_WS, new_cloud)
            if X_G2_pose_array:
                # rospy.loginfo('{} poses from XG2'.format(len(X_G2_pose_array.poses)))
                pose_array.poses += X_G2_pose_array.poses
                debug_pose_array.poses += debuggin_pose_array.poses 

            # rotate around x
            theta_r = R.from_euler('x', theta, degrees=False)
            X_G3 = X_G2.dot(theta_r.as_matrix())
            X_G3_pose_array, debuggin_pose_array = cost(R.from_matrix(X_G3), p_WS, new_cloud)
            if X_G3_pose_array:
                # rospy.loginfo('{} poses from XG3'.format(len(X_G3_pose_array.poses)))
                pose_array.poses += X_G3_pose_array.poses
            #     debug_pose_array.poses += debuggin_pose_array.poses 
        
        if len(pose_array.poses) > 50:
            break

    print('generated {} antipodal grasps!'.format(len(pose_array.poses)))
    pose_array.header.stamp = rospy.Time.now()
    antipodal_pub.publish(pose_array)

    debug_pose_array.header.stamp = rospy.Time.now()
    local_basis_pub.publish(debug_pose_array)


    executionTime = (time.time() - startTime)
    with open('grasps_generation_time.txt', 'a+') as f:
        f.write('Execution time in seconds: ' + str(executionTime) + ' for 1 object\n')
    rospy.loginfo('Execution time in seconds: ' + str(executionTime))

    resp = AntipodalGraspResponse([pose_array])
    return resp

def cost(X_G, p_WS, cloud):
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

    debugging_pa = PoseArray()
    X_G = X_G.as_matrix()
    q = R.from_matrix(X_G).as_quat()
    r_g = np.vstack((np.hstack((X_G, p_WS.reshape(3,1))), [0, 0, 0 ,1]))
    p_WS_ = r_g.dot(np.array([-0.25, 0., 0, 1]))

    theta_y = R.from_euler('y', 180., degrees=True)
    X_G = X_G.dot(theta_y.as_matrix())

    rigid_transform = np.vstack((np.hstack((X_G, p_WS.reshape(3,1))), [0, 0, 0 ,1]))
    

    quat = R.from_matrix(X_G).as_quat()
    grasp = rigid_transform.dot(np.array([-0.25, 0., 0, 1]))
    # grasp_box = rigid_transform.dot(np.array([0, 0., 0, 1]))    
    
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
        pa = PoseArray()
        pa.poses.append(Pose(Point(grasp[0], grasp[1], grasp[2]), Quaternion(quat[0], quat[1], quat[2], quat[3])))
        pa.poses.append(Pose(Point(p_WS_[0], p_WS_[1], p_WS_[2]), Quaternion(q[0], q[1], q[2], q[3])))
        new_pa, indices = filter_grasps_downwards('base_footprint', pa, threshold=0.8)

        debugging_pa.poses.append(Pose(Point(p_WS[0], p_WS[1], p_WS[2]), Quaternion(q[0], q[1], q[2], q[3])))
        # theta_r = R.from_euler('y', 180., degrees=True)
        # X_G2 = X_G.dot(theta_r.as_matrix())
        # trans = np.vstack((np.hstack((X_G2, p_WS.reshape(3,1))), [0, 0, 0 ,1]))
        # grasp2 = trans.dot(np.array([0.20, 0., 0, 1]))
        # quat2 = R.from_matrix(X_G2).as_quat()
        # pa.poses.append(Pose(Point(grasp2[0], grasp2[1], grasp2[2]), Quaternion(quat2[0], quat2[1], quat2[2], quat2[3])))
        return new_pa, debugging_pa
        # rospy.loginfo('poses number is {}'.format(len(pa.poses)))

        # return shift_grasps_ee(pa)
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
    return False, False

def shift_grasps_ee(original_pose_array, delta=-0.05):
    """
    Arguments 
        original_pose_array {PoseArray}
        delta {float} -- negative offset by which grasps are moved backwards 
    """


    pose_array = deepcopy(original_pose_array)
    # pose_array = tf_transform('base_footprint', pose_array).target_pose_array
    for pose in pose_array.poses:
        # print('x: {}, y: {}, z: {}'.format(pose.position.x, pose.position.y, pose.position.z))
        q_orien = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        euler = euler_from_quaternion(q_orien)
        euler_m = euler_matrix(*euler)

        # print('{} {} {}'.format(pose.position.x, pose.position.y, pose.position.z))
        # print('{} {} {}'.format(euler_m[0, 0], euler_m[0, 1], euler_m[0, 2]))
        
        pose.position.x += delta * euler_m[0, 0]
        pose.position.y += delta * euler_m[1, 0]
        pose.position.z += delta * euler_m[2, 0]
        # print('x: {}, y: {}, z: {}'.format(pose.position.x, pose.position.y, pose.position.z))

    # grasps are rotated by in camera frame, so we have to convert them into base_footprint frame
    # pose_array = tf_transform('base_footprint', pose_array).target_pose_array
    return pose_array

def filter_grasps_downwards(frame_id, pose_array_map, threshold=0.5):
    pose_array = PoseArray(header=Header(frame_id=frame_id, stamp=rospy.get_rostime()))

    map_rot_q = quaternion_from_euler(0., math.pi/2., 0., 'sxyz')
    map_rot_e = euler_from_quaternion(map_rot_q)
    map_rot_m = euler_matrix(*map_rot_e)  # TODO: map_rot_m[:, 0] might just be [[0], [0], [-1]] i.e. x facing down in z-direction

    indices = []
    # filter for grasps that point downwards
    for i, pose_map in enumerate(pose_array_map.poses):
        quat_map = np.array([
            pose_map.orientation.x,
            pose_map.orientation.y,
            pose_map.orientation.z,
            pose_map.orientation.w
        ])

        rot_e = euler_from_quaternion(quat_map)
        rot_m = euler_matrix(*rot_e)

        overlap = np.dot(rot_m[:,0], map_rot_m[:,0])
        if overlap > threshold:
            pose_array.poses.append(pose_map)
            indices.append(i)
    
    return pose_array, indices


def publish_cloud(pcd):
    ros_pcd = o3dpc_to_rospc(pcd, 'base_footprint', rospy.Time.now())
    axis_aligned_pcd.publish(ros_pcd)


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
    obj_basis_pub = rospy.Publisher('/obj_basis_pub', PoseStamped, queue_size=1, latch=True)
    # marker_pub = rospy.Publisher('/bb_corners', MarkerArray, queue_size = 8)
    # one_marker_pub = rospy.Publisher('/box', Marker, queue_size = 1, latch=True)
    local_basis_pub = rospy.Publisher('/all_local_basis', PoseArray, queue_size=1, latch=True)
    axis_aligned_pcd = rospy.Publisher('/axis_aligned_pcd', PointCloud2, queue_size=1, latch=True)

    print('Ready to generate_grasp_candidate_antipodal!')
    rospy.spin()

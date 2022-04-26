#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import actionlib
import tf2_ros
import ros_numpy

from pick_up_object.srv import GenerateGrasps, TfTransform, TfTransformRequest, DetectObjects
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from pcl_manipulation.srv import Euclidian
from pick_up_object.srv import PosesAroundObj, PointcloudReconstruction, PointcloudReconstructionRequest, PcdLocalFrames, AntipodalGrasp, AntipodalGraspRequest



def detect_objs():
    """
        Calls detection server (Mask-RCNN)

        Return:
            detection {DetectObjects}
    """

    print('waiting for detect_objects')
    rospy.wait_for_service('detect_objects', timeout=10)
    try:
        detect = rospy.ServiceProxy('detect_objects', DetectObjects)
        resp = detect()
        print('detection done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def detect_clusters(pcl):
    """
        Calls clustering service

        Return:
            clusters {Euclidean}
    """

    print('waiting for detect_objects')
    rospy.wait_for_service('/pcl_manipulation/cluster', timeout=10)
    try:
        detect = rospy.ServiceProxy('/pcl_manipulation/cluster', Euclidian)
        resp = detect(pcl)
        print('Clustering done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def generate_grasps(full_pcl, objs_pcl):
    """
    Arguments:
        full_pcl {PointCloud2} -- full point cloud of the scene
        objs_pcl {PointCloud2[]} -- list with each segmented object pointcloud
    
    Return:
        grasps {GenerateGrasps} -- array of grasps
    """

    print('waiting for generate_grasps_server')
    rospy.wait_for_service('generate_grasps_server', timeout=20)
    try:
        grasp_poses = rospy.ServiceProxy('generate_grasps_server', GenerateGrasps)
        resp = grasp_poses(full_pcl, objs_pcl)
        print('generates grasps done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def clear_octomap():
    """
        Clears octomap using clear_octomap server
    """

    print('waiting for clear_octomap')
    rospy.wait_for_service('clear_octomap', timeout=10)
    try:
        clear_octo = rospy.ServiceProxy('clear_octomap', Empty)
        response = clear_octo()
        print('clearing octomap done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def tf_transform(target_frame, pose_array=None, pointcloud=None):
    """
    Arguments:
        target_frame {frame_id} -- frame to transform to
        pose_array {PoseArray} -- array of poses
        pointcloud {PointCloud2}
    
    Returns:
        response {TfTransformResponse} -- target_pose_array {PoseArray}, target_pointcloud {PointCloud2}
    """

    assert pose_array is not None or pointcloud is not None

    # print('about to transform to ' + target_frame)
    rospy.wait_for_service('tf_transform', timeout=10)
    try:
        tf_transform_srv = rospy.ServiceProxy('tf_transform', TfTransform)
        request = TfTransformRequest()
        if pose_array is not None:
            request.pose_array = pose_array
        if pointcloud is not None:
            request.pointcloud = pointcloud
        request.target_frame.data = target_frame
        response = tf_transform_srv(request)
        # print('transform done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def to_frame_pose(pose, source_frame='xtion_depth_optical_frame', target_frame='base_footprint'):
    """
    Arguments:
        pose {Pose} -- pose to convert
        source_frame {frame id} -- original coordinate frame
        target_frame {frame id} -- target coordinate frame
    Return:
        pose {Pose} -- target pose
    """
    tfBuffer = tf2_ros.Buffer()
    # remove '/' from source_frame and target frame to avoid tf2.InvalidArgumentException
    source_frame = source_frame.replace('/', '')
    target_frame = target_frame.replace('/', '')
    try:
        transformation = tfBuffer.lookup_transform(target_frame, source_frame,
        rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
    
    pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=pose), transformation).pose
    return pose

def add_collision_object(object_cloud, planning_scene, num_primitives = 200, id='object'):
    """
    adds collision object to the planning scene created by moveit

    Arguments:
        object_cloud {PointCloud2} -- pointcloud of the object
        num_primitives -- number of primitives (squares) to create the collision object

    Returns:
        co {CollisionObject} -- collision object
    """
    target_cloud = tf_transform(target_frame='base_footprint', pointcloud=object_cloud).target_pointcloud

    # pcl = np.fromstring(object_cloud.data, np.float32)
    # print('pcl shape fromsring = {}'.format(pcl.shape))
    # pcl = pcl.reshape(object_cloud.height, object_cloud.width, -1)
    # print('pcl shape after reshape = {}'.format(pcl.shape))
    # cloud_obj = np.zeros(pcl.shape[0], dtype=[
    #     ('x', np.float32),
    #     ('y', np.float32),
    #     ('z', np.float32)
    # ])
    # cloud_obj['x'] = pcl[:,:,0].flatten()
    # cloud_obj['y'] = pcl[:,:,1].flatten()
    # cloud_obj['z'] = pcl[:,:,2].flatten()
    pcl = ros_numpy.numpify(target_cloud)
    cloud_obj = np.concatenate( (pcl['x'].reshape(-1,1), pcl['y'].reshape(-1,1), pcl['z'].reshape(-1,1)), axis=1)

    # add collision object to planning scene
    co = CollisionObject()
    co.id = id

    # create collision object
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.07, 0.07, 0.07]

    indices = np.random.choice(range(pcl.shape[0]), size=np.min([num_primitives, pcl.shape[0]]), replace=False)
    pose_array = PoseArray()

    for i in indices:
        x,y,z = cloud_obj[i]
        primitive_pose = Pose()
        primitive_pose.position.x = x
        primitive_pose.position.y = y
        primitive_pose.position.z = z
        primitive_pose.orientation.x = 0
        primitive_pose.orientation.y = 0
        primitive_pose.orientation.z = 0
        primitive_pose.orientation.w = 1

        co.primitives.append(primitive)
        pose_array.poses.append(primitive_pose)

        #hacky
        # x,y,z = cloud_obj[i]
        # pp = Pose()
        # pp.position.x = x
        # pp.position.y = y
        # pp.position.z = z+0.15
        # pp.orientation.x = 0
        # pp.orientation.y = 0
        # pp.orientation.z = 0
        # pp.orientation.w = 1

        # co.primitives.append(primitive)
        # pose_array.poses.append(pp)

    co.header = target_cloud.header
    co.primitive_poses = pose_array.poses

    planning_scene.add_object(co)
    return co

def play_motion_action(action='home'):
    """
    Arguments:
        action {String} -- predefined actions

    Return:
        bool -- action statusf 
    """


    pm_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    pm_client.wait_for_server()
    goal = PlayMotionGoal()
    goal.motion_name = action
    goal.skip_planning = False
    goal.priority = 0  # Optional

    print("Sending goal with motion: " + action)
    pm_client.send_goal(goal)

    print("Waiting for result...")
    action_ok = pm_client.wait_for_result(rospy.Duration(30.0))

    state = pm_client.get_state()
    if action_ok:
        print("Action finished succesfully with state: " + str(actionlib.GoalStatus.to_string(state)))
    else:
        rospy.logwarn("Action failed with state: " + str(actionlib.GoalStatus.to_string(state)))
    
    print(action_ok and state == actionlib.GoalStatus.SUCCEEDED)
    return action_ok and state == actionlib.GoalStatus.SUCCEEDED


def get_poses_around_obj(obj):
    rospy.wait_for_service('find_poses_around_obj', timeout=10)
    try:
        find_poses = rospy.ServiceProxy('find_poses_around_obj', PosesAroundObj)
        resp = find_poses(obj).target_poses
        print('we have the poses!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def generate_antipodal_grasps(req):
    rospy.wait_for_service('generate_grasp_candidate_antipodal', timeout=10)
    try:
        grasps_server = rospy.ServiceProxy('generate_grasp_candidate_antipodal', AntipodalGrasp)
        resp = grasps_server(req)
        print('generate_grasp_candidate_antipodal done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def compute_local_basis(reconstructed_pcd):
    rospy.wait_for_service('pcd_local_frames', timeout=10)
    try:
        reconstruct_pcd = rospy.ServiceProxy('pcd_local_frames', PcdLocalFrames)
        resp = reconstruct_pcd(reconstructed_pcd).local_basis
        print('local basis calculation done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def reconstruct_pcd(pointcloud_array):
    rospy.wait_for_service('reconstruct_pointcloud', timeout=10)
    try:
        reconstruct_pcd = rospy.ServiceProxy('reconstruct_pointcloud', PointcloudReconstruction)
        resp = reconstruct_pcd(pointcloud_array).full_reconstructed_pcd
        print('reconstruction done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

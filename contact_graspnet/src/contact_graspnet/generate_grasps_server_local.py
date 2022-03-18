#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import rospy
import tensorflow.compat.v1 as tf
import config_utils
import rospkg
import ros_numpy

from contact_graspnet.srv import GenerateGrasps, GenerateGraspsResponse
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from utils.transformations import * # python3 can't import tf.transformations
from copy import deepcopy
from contact_graspnet.utils import tf_transform
from contact_grasp_estimator import GraspEstimator

# ---------------------
# Set tensorflow params
# ---------------------

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(BASE_DIR))

tf.disable_eager_execution()
tf.disable_v2_behavior()
physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)



# Create a session
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
config.allow_soft_placement = True
# config.gpu_options.per_process_gpu_memory_fraction=0.2 # applies to my GPU -- 3060 RTX
sess = tf.Session(config=config)

# ---------------------------------
# set contact graspnet model config
# ---------------------------------
model_path = os.path.join(rospkg.RosPack().get_path('contact_graspnet'), 'checkpoints')
print(model_path)

parser = argparse.ArgumentParser()
parser.add_argument('--ckpt_dir', default=model_path+'/scene_test_2048_bs3_hor_sigma_001', help='Log dir [default: ../../checkpoints/scene_test_2048_bs3_hor_sigma_001]')
parser.add_argument('--np_path', default='test_data/7.npy', help='Input data: npz/npy file with keys either "depth" & camera matrix "K" or just point cloud "pc" in meters. Optionally, a 2D "segmap"')
parser.add_argument('--png_path', default='', help='Input data: depth map png in meters')
parser.add_argument('--K', default=None, help='Flat Camera Matrix, pass as "[fx, 0, cx, 0, fy, cy, 0, 0 ,1]"')
parser.add_argument('--z_range', default=[0.2,1.8], help='Z value threshold to crop the input point cloud')
parser.add_argument('--local_regions', action='store_true', default=False, help='Crop 3D local regions around given segments.')
parser.add_argument('--filter_grasps', action='store_true', default=False,  help='Filter grasp contacts according to segmap.')
parser.add_argument('--skip_border_objects', action='store_true', default=False,  help='When extracting local_regions, ignore segments at depth map boundary.')
parser.add_argument('--forward_passes', type=int, default=1,  help='Run multiple parallel forward passes to mesh_utils more potential contact points.')
parser.add_argument('--segmap_id', type=int, default=0,  help='Only return grasps of the given object id')
parser.add_argument('--arg_configs', nargs="*", type=str, default=[], help='overwrite config parameters')
parser.add_argument('__name', nargs="*", type=str, default=[], help='overwrite config parameters') # this parameter simply allows ROS to launch the file with roslaunch
FLAGS = parser.parse_args()

global_config = config_utils.load_config(FLAGS.ckpt_dir, batch_size=FLAGS.forward_passes, arg_configs=FLAGS.arg_configs)

print(str(global_config))
print('pid: %s'%(str(os.getpid())))

checkpoint_dir = FLAGS.ckpt_dir
print(checkpoint_dir)

# Build the model
grasp_estimator = GraspEstimator(global_config)
grasp_estimator.build_network()

# Add ops to save and restore all the variables.
saver = tf.train.Saver(save_relative_paths=True)

# publisher of grasps. 200 grasps generated
grasps_all = rospy.Publisher('grasps_all', PoseArray, queue_size=200, latch=True)

print('loading weights...')
grasp_estimator.load_weights(sess, saver, checkpoint_dir, mode='test')


# callback function in generating grasps service
def generate_grasps(req):
    """
    Arguments:
        req {GenerateGrasps} -- message container full_pcl and objs_pcl
    Return:
        response {GenerateGraspsResponse} -- number_objects, all_grasp_poses, grasp_contact_points, object_heights, all_scores 
    """
    
    frame_id = req.full_pcl.header.frame_id

    # reshape pcl to be in the right form to be processed
    pcl = ros_numpy.numpify(req.full_pcl)
    pcl_np = np.concatenate( (pcl['x'].reshape(-1,1), pcl['y'].reshape(-1,1), pcl['z'].reshape(-1,1)), axis=1)

    # reshape each object pcl
    objs_np = {}
    for i, object_pcl in enumerate(req.objects_pcl):
        obj_np = ros_numpy.numpify(object_pcl)
        obj_np = np.concatenate( (obj_np['x'].reshape(-1,1), obj_np['y'].reshape(-1,1), obj_np['z'].reshape(-1,1)), axis=1)
        objs_np[i] = obj_np

    # camera matrix from topic '/xtion/depth_registered/camera_info'
    # K = [522.1910329546544, 0.0, 320.5, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 1.0]


    response = GenerateGraspsResponse()
    pose_array_full = None

    print('Generating Grasps...')
    pred_grasps_cam, scores, contacts, openings = grasp_estimator.predict_scene_grasps(sess, pc_full=pcl_np, 
                                                                                        pc_segments=objs_np, local_regions=True, 
                                                                                        filter_grasps=True, forward_passes=1)
    # loop through each object's pcl and adjust predicted grasps                                                                                  
    for i in range(len(req.objects_pcl)):
        pc_seg_map = tf_transform('map', pointcloud=req.objects_pcl[i]).target_pointcloud
        pc_seg_map = ros_numpy.numpify(pc_seg_map)
        max_z, min_z = np.max(pc_seg_map['z']), np.min(pc_seg_map['z'])
        
        object_height = max_z - min_z
        response.object_heights.append(object_height)

        if i in pred_grasps_cam:
            pose_array = convert_opencv_to_tiago(frame_id=frame_id, grasps=pred_grasps_cam[i])
            pose_array = shift_grasps_ee(pose_array)
            # pose_array = tf_transform('base_footprint', pose_array).target_pose_array
            response.all_grasp_poses.append(pose_array)
            response.all_scores.append(np.nanmean(scores[i]) if np.nanmean(scores[i]) != np.nan else 0.)
            if np.isnan(np.nanmean(scores[i])):
                print(scores[i])
                print(np.nanmean(scores[i]))
                print('nan scores: {}, object: {}'.format(scores[i], i))

            if pose_array_full is None:
                pose_array_full = deepcopy(pose_array)
            else:
                pose_array_full.poses += pose_array.poses
        else: # empty grasps
            response.all_grasp_poses.append(PoseArray())
            response.all_scores.append(0.)

    if pose_array_full:
        grasps_all.publish(pose_array_full)

    print('generated grasps!')
    return response


def convert_opencv_to_tiago(frame_id, grasps):
    """
    Arguments:
        frame_id {frame id} -- target coordinate frame to convert to
        grasps {array} -- array containing all grasps for an object
    Return:
        pose_array {PoseArrray} -- PoseArray of grasps
    """

    # change coords from NVLabs to Tiago gripper
    pose_array = PoseArray(header=Header(frame_id=frame_id, stamp=rospy.get_rostime()))

    q_rot = quaternion_from_euler(-math.pi/2., -math.pi/2., 0., 'sxyz') # rotation 1
    q_rot_180 = quaternion_from_euler(math.pi, 0., 0., 'sxyz') # rotation 2

    for grasp in grasps:
        quat = quaternion_from_matrix(grasp)
        quat = quaternion_multiply(quat, q_rot)

        pose = Pose()
        pose.position.x = grasp[0,3]
        pose.position.y = grasp[1,3]
        pose.position.z = grasp[2,3]

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        pose_180 = Pose()
        pose_180.position = pose.position

        quat = quaternion_multiply(quat, q_rot_180)
        pose_180.orientation.x = quat[0]
        pose_180.orientation.y = quat[1]
        pose_180.orientation.z = quat[2]
        pose_180.orientation.w = quat[3]

        pose_array.poses.append(pose)
        pose_array.poses.append(pose_180)
    return pose_array


def shift_grasps_ee(original_pose_array, delta=-0.025):
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
    pose_array = tf_transform('base_footprint', pose_array).target_pose_array
    return pose_array


if __name__ == '__main__':
    rospy.init_node('contact_graspnet_server', anonymous=True)
    try:
        s = rospy.Service('generate_grasps_server', GenerateGrasps, generate_grasps)
        print("Ready to generate grasps poses")
    except KeyboardInterrupt:
        sys.exit()
    rospy.spin()

#!/usr/bin/python3

import os
import sys
import argparse
import numpy as np
import time
import cv2
import rospy
from contact_graspnet.srv import GenerateGraspsSrv
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import CameraInfo
import ros_numpy

import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
tf.disable_v2_behavior()

physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(BASE_DIR))
import config_utils

from contact_grasp_estimator import GraspEstimator

# Create a session
config = tf.ConfigProto()
config.gpu_options.allow_growth = False
config.allow_soft_placement = True
config.gpu_options.per_process_gpu_memory_fraction=0.25
sess = tf.Session(config=config)

parser = argparse.ArgumentParser()
parser.add_argument('--ckpt_dir', default='../../checkpoints/scene_test_2048_bs3_hor_sigma_001', help='Log dir [default: ../../checkpoints/scene_test_2048_bs3_hor_sigma_001]')
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
FLAGS = parser.parse_args()

global_config = config_utils.load_config(FLAGS.ckpt_dir, batch_size=FLAGS.forward_passes, arg_configs=FLAGS.arg_configs)

print(str(global_config))
print('pid: %s'%(str(os.getpid())))

checkpoint_dir = FLAGS.ckpt_dir

# Build the model
grasp_estimator = GraspEstimator(global_config)
grasp_estimator.build_network()

# Add ops to save and restore all the variables.
saver = tf.train.Saver(save_relative_paths=True)

# Load weights
print('loading weights...')
grasp_estimator.load_weights(sess, saver, checkpoint_dir, mode='test')
print('weights loaded!')

# publisher of grasps. 200 grasps generated
graps_publisher = rospy.Publisher('grasps_all', PoseArray, queue_size=10, latch=True)



def generate_grasps(req):

    # reshape pcl to be in the right form to be processed
    pcl = ros_numpy.numpify(req.full_pcl)
    pcl = np.concatenate( (pcl['x'].reshape(-1,1), pcl['y'].reshape(-1,1), pcl['z'].reshape(-1,1)), axis=1) # 3x_ matrix


    # camera matrix from topic '/xtion/depth_registered/camera_info'
    K = [522.1910329546544, 0.0, 320.5, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 1.0]

    # pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(depth=pcl, K=K, segmap=None, rgb=None,
    #                                             skip_border_objects=False)

    print('Generating Grasps...')
    # pred_grasps_cam, scores, contact_pts, _ = grasp_estimator.predict_scene_grasps(sess, pcl, pc_segments=pc_segments,
    #                                             local_regions=True, filter_grasps=True, forward_passes=1)

    pred_grasps_cam, scores, contact_pts, gripper_openings =  grasp_estimator.predict_grasps(sess, pcl, constant_offset=False, 
                                                                    convert_cam_coords=True, forward_passes=1)

    return 1




if __name__ == '__main__':
    rospy.init_node('contact_graspnet_server', anonymous=True)
    try:
        s = rospy.Service('generate_grasps_service', GenerateGraspsSrv, generate_grasps)
        print("Ready to generate grasps poses")
    except KeyboardInterrupt:
        sys.exit()
    rospy.spin()

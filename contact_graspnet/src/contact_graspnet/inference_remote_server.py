import os
import sys
import argparse
import numpy as np
import time
# import glob
import cv2
import tensorflow.compat.v1 as tf
from mlsocket import MLSocket
import socket
from time import sleep
import errno
from npsocket import SocketNumpyArray
import struct
import json
import pickle

tf.disable_eager_execution()
physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(BASE_DIR))
import config_utils
from data import regularize_pc_point_count, depth2pc, load_available_input_data

from contact_grasp_estimator import GraspEstimator
# from visualization_utils import visualize_grasps, show_image

parser = argparse.ArgumentParser()
parser.add_argument('--ckpt_dir', default='../../checkpoints/tiago_weights_0', help='Log dir [default: checkpoints/scene_test_2048_bs3_hor_sigma_001]')
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

# Build the model
grasp_estimator = GraspEstimator(global_config)
grasp_estimator.build_network()

# Add ops to save and restore all the variables.
saver = tf.train.Saver(save_relative_paths=True)

# Create a session
config = tf.ConfigProto()
config.gpu_options.allow_growth = False
config.allow_soft_placement = True
config.gpu_options.per_process_gpu_memory_fraction = 0.2
sess = tf.Session(config=config)

# Load weights
# grasp_estimator.load_weights(sess, saver, '../../checkpoints/training4', mode='test') 
grasp_estimator.load_weights(sess, saver, '../../checkpoints/scene_test_2048_bs3_hor_sigma_001', mode='test') 

      
        
if __name__ == "__main__":

    # print(str(global_config))
    print('pid: %s'%(str(os.getpid())))

    HOST = '158.176.76.55'
    REMOTE_HOST = '31.205.214.65'
    PORT = 9997


    with MLSocket() as s:
        s.bind((HOST, PORT))
        while True:
            s.listen()
            conn, address = s.accept()

            with conn:
                pcl_msg = conn.recv(1024) # This will block until it receives all the data send by the client, with the step size of 1024 bytes.
                print(pcl_msg.shape)
                # pc_segments = {}
                # segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(pcl_msg, K=None)
                
                # if segmap is None and (local_regions or filter_grasps):
                #     raise ValueError('Need segmentation map to extract local regions or filter grasps')

                # cam_K = [522.1910329546544, 0.0, 320.5, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 1.0]
                # K=np.ndarray((3,3), buffer=np.array(cam_K))
                # if pc_full is None:
                #     print('Converting depth to point cloud(s)...')
                    # pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(pcl_msg, K=K, segmap=None, rgb=None, skip_border_objects=False)

                print('Generating Grasps...')
                # pred_grasps_cam, scores, contact_pts, _ = grasp_estimator.predict_scene_grasps(sess, pc_full, pc_segments=pc_segments, 
                #                                                                           local_regions=True, filter_grasps=True, forward_passes=1)
                
                pred_grasps_cam, scores, pred_points, gripper_openings = grasp_estimator.predict_grasps(sess, pcl_msg, constant_offset=False, convert_cam_coords=True, forward_passes=1)

                # send back grasps to client
                conn.send(pred_grasps_cam)

        
                
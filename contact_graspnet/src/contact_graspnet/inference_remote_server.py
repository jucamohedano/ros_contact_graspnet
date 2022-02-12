import os
import sys
import argparse
import numpy as np
import time
# import glob
import cv2
import tensorflow.compat.v1 as tf
from mlsocket import MLSocket

HOST = '158.176.76.55'
PORT = 65432

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
parser.add_argument('--ckpt_dir', default='../checkpoints/scene_test_2048_bs3_hor_sigma_001', help='Log dir [default: checkpoints/scene_test_2048_bs3_hor_sigma_001]')
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
grasp_estimator.load_weights(sess, saver, '../checkpoints/scene_test_2048_bs3_hor_sigma_001', mode='test')

def inference(global_config, checkpoint_dir, pcl, K=None, local_regions=True, skip_border_objects=False, filter_grasps=True, segmap_id=None, z_range=[0.2,1.8], forward_passes=1):
    """
    Predict 6-DoF grasp distribution for given model and input data
    
    :param global_config: config.yaml from checkpoint directory
    :param checkpoint_dir: checkpoint directory
    :param input_paths: .png/.npz/.npy file paths that contain depth/pointcloud and optionally intrinsics/segmentation/rgb
    :param K: Camera Matrix with intrinsics to convert depth to point cloud
    :param local_regions: Crop 3D local regions around given segments. 
    :param skip_border_objects: When extracting local_regions, ignore segments at depth map boundary.
    :param filter_grasps: Filter and assign grasp contacts according to segmap.
    :param segmap_id: only return grasps from specified segmap_id.
    :param z_range: crop point cloud at a minimum/maximum z distance from camera to filter out outlier points. Default: [0.2, 1.8] m
    :param forward_passes: Number of forward passes to run on each point cloud. Default: 1
    """

    # Load weights
    # grasp_estimator.load_weights(sess, saver, checkpoint_dir, mode='test')
    # pc_segments = {}
    # segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(pcl, K=K)
    
    # if segmap is None and (local_regions or filter_grasps):
    #     raise ValueError('Need segmentation map to extract local regions or filter grasps')

    # if pc_full is None:
    #     print('Converting depth to point cloud(s)...')
    #     pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(depth, cam_K, segmap=segmap, rgb=rgb,
    #                                                                             skip_border_objects=skip_border_objects, z_range=z_range)

    # print('Generating Grasps...')
    # pred_grasps_cam, scores, contact_pts, _ = grasp_estimator.predict_scene_grasps(sess, pc_full, pc_segments=pc_segments, 
    #                                                                                 local_regions=local_regions, filter_grasps=filter_grasps, forward_passes=forward_passes)
    K = [522.1910329546544, 0.0, 320.5, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 1.0]
    pred_grasps_cam, scores, pred_points, gripper_openings = grasp_estimator.predict_grasps(sess, pcl, constant_offset=False, convert_cam_coords=True, forward_passes=1)       

        
if __name__ == "__main__":

    address = ''
    data = []
    print(str(global_config))
    print('pid: %s'%(str(os.getpid())))

    with MLSocket() as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, address = s.accept()
        with conn:
            data = conn.recv(1024)

            # pcl = inference(global_config, FLAGS.ckpt_dir, data, z_range=eval(str(FLAGS.z_range)),
            #             K=FLAGS.K, local_regions=FLAGS.local_regions, filter_grasps=FLAGS.filter_grasps, segmap_id=FLAGS.segmap_id, 
            #             forward_passes=FLAGS.forward_passes, skip_border_objects=FLAGS.skip_border_objects)
            pred_grasps_cam, scores, pred_points, gripper_openings = grasp_estimator.predict_grasps(sess, data, constant_offset=False, convert_cam_coords=True, forward_passes=1)
            conn.send(pred_grasps_cam)


        
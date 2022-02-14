#!/usr/bin/python3

import os
import sys
import argparse
import numpy as np
import time
import cv2
import rospy
from contact_graspnet.srv import GenerateGraspsSrv, TfTransform, TfTransformRequest
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import CameraInfo
import ros_numpy
from mlsocket import MLSocket
import socket
from npsocket import SocketNumpyArray
import struct
import pickle
from numpySocket import NumpySocket
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from utilities.transformations import *

# publisher of grasps. 200 grasps generated
grasps_all_pub = rospy.Publisher('grasps_all', PoseArray, queue_size=10, latch=True)


REMOTE_HOST = '158.176.76.55'
PORT = 9997

def generate_grasps(req):
    frame_id = req.full_pcl.header.frame_id

    # reshape pcl to be in the right form to be processed
    pcl = ros_numpy.numpify(req.full_pcl)
    pcl = np.concatenate( (pcl['x'].reshape(-1,1), pcl['y'].reshape(-1,1), pcl['z'].reshape(-1,1)), axis=1) # 3x_ matrix

    # camera matrix from topic '/xtion/depth_registered/camera_info'
    K = [522.1910329546544, 0.0, 320.5, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 1.0]

    grasps = []
    with MLSocket() as s:
        s.connect((REMOTE_HOST, PORT)) # Connect to the port and host
        s.sendall(pcl) # After sending the data, it will wait until it receives the reponse from the server
        grasps = s.recv(1024)
    # print(grasps)
    grasps_pose_array = convert_opencv_to_tiago(frame_id, grasps)
    grasps_all_pub.publish(grasps_pose_array)

    return 1

def convert_opencv_to_tiago(frame_id, grasps):
    # change coords from NVLabs to Tiago gripper
    pose_array = PoseArray(header=Header(frame_id=frame_id, stamp=rospy.get_rostime()))

    q_rot = quaternion_from_euler(-math.pi/2., -math.pi/2., 0., 'sxyz')
    q_rot_180 = quaternion_from_euler(math.pi, 0., 0., 'sxyz')

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


if __name__ == '__main__':
    rospy.init_node('contact_graspnet_server', anonymous=True)
    try:
        s = rospy.Service('generate_grasps_service', GenerateGraspsSrv, generate_grasps)
        print("Ready to generate grasps poses")
    except KeyboardInterrupt:
        sys.exit()
    rospy.spin()

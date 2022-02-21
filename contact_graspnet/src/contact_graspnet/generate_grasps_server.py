#!/usr/bin/python3

import os
import sys
import numpy as np
import rospy
from contact_graspnet.srv import GenerateGraspsSrv, GenerateGraspsSrvResponse, TfTransform, TfTransformRequest
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import CameraInfo
import ros_numpy
from mlsocket import MLSocket
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
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

    # process each object pcl
    pc_segments={}
    for i, object_pcl in enumerate(req.objects_pcl):
        obj_np = ros_numpy.numpify(object_pcl)
        obj_np = np.concatenate( (obj_np['x'].reshape(-1,1), obj_np['y'].reshape(-1,1), obj_np['z'].reshape(-1,1)), axis=1)
        pc_segments[i] = obj_np

    # camera matrix from topic '/xtion/depth_registered/camera_info'
    K = [522.1910329546544, 0.0, 320.5, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 1.0]

    grasps = []
    with MLSocket() as s:
        s.connect((REMOTE_HOST, PORT)) # Connect to the port and host
        s.sendall(pcl) # After sending the data, it will wait until it receives the reponse from the server
        s.sendall(pc_segments)
        grasps = s.recv(1024)

    #TODO: update remote server to return scores
    
    response = GenerateGraspsSrvResponse()
    pose_array_full = None


    for i in range(len(req.objects_pcl)):
        pc_seg_map = tf_transform('map', pointcloud=req.objects_pcl[i]).target_pointcloud
        pc_seg_map = ros_numpy.numpify(pc_seg_map)
        
        if i in grasps:
            pose_array = convert_nv_labs_to_tiago(frame_id, grasps[i])

            response.all_grasp_poses.append(pose_array)


            if pose_array_full is None:
                pose_array_full = deepcopy(pose_array) # first iteration pose_array_full is None
            else:
                pose_array_full.poses += pose_array.poses
        else: # empty grasps
            response.all_grasp_poses.append(PoseArray())
            response.all_scores.append(0.)

    if pose_array_full:
        grasps_all_pub.publish(pose_array_full)


    return response

def convert_nv_labs_to_tiago(frame_id, grasps):
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

def tf_transform(target_frame, pose_array=None, pointcloud=None):
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

if __name__ == '__main__':
    rospy.init_node('contact_graspnet_server', anonymous=True)
    try:
        s = rospy.Service('generate_grasps_server', GenerateGraspsSrv, generate_grasps)
        print("Ready to generate grasps poses")
    except KeyboardInterrupt:
        sys.exit()
    rospy.spin()

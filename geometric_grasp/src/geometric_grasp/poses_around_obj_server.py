#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np

from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from geometric_grasp.srv import PosesAroundObj, PosesAroundObjResponse
from geometric_grasp.utils import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d 

def get_poses(msg):
    resp = PosesAroundObjResponse()
    
    o3d_cloud = convertCloudFromRosToOpen3d(msg.object_cloud)
    mean, _ = o3d_cloud.compute_mean_and_covariance()
    
    pose_array = compute_poses_around_object(mean, msg.object_cloud.header.frame_id, d=0.28)
    resp.target_poses = pose_array
    return resp


def compute_poses_around_object(pose, frame_id, d=0.25):
    """
    Arguments
        pose {Pose} - mean pose of the object
        d {float} - offset
    Return
        set of poses around the object
    """
    # set of hard coded positions around the object
    pose_array = PoseArray()
    pose_array.header.frame_id = frame_id
    pose_array.header.stamp = rospy.Time.now()

    ## FRONT POSE
    # in_front_pose = Pose()
    # in_front_pose.position = Point(pose[0]-0.4, pose[1], pose[2]+0.3)
    
    # # vectors to mean
    # new_orientation = np.array([pose[0], pose[1], pose[2]]) - \
    #                 np.array([
    #                 in_front_pose.position.x, 
    #                 in_front_pose.position.y, 
    #                 in_front_pose.position.z])
    # new_quaternion = pose_from_vector3D(new_orientation)
    # in_front_pose.orientation = new_quaternion
    # pose_array.poses.append(in_front_pose)

    ## LEFT POSE
    right_pose = Pose()
    right_pose.position = Point(pose[0], pose[1]+d, pose[2]+d)
    
    # vectors to mean
    right_new_orientation = np.array([pose[0], pose[1], pose[2]]) - \
                    np.array([
                    right_pose.position.x, 
                    right_pose.position.y, 
                    right_pose.position.z])
    right_new_quaternion = pose_from_vector3D(right_new_orientation)
    right_pose.orientation = right_new_quaternion
    pose_array.poses.append(right_pose)

    # RIGHT POSE
    left_pose = Pose()
    left_pose.position = Point(pose[0], pose[1]-d, pose[2]+d)
    
    # vectors to mean
    left_new_orientation = np.array([pose[0], pose[1], pose[2]]) - \
                    np.array([
                    left_pose.position.x, 
                    left_pose.position.y, 
                    left_pose.position.z])
    left_new_quaternion = pose_from_vector3D(left_new_orientation)
    left_pose.orientation = left_new_quaternion
    pose_array.poses.append(left_pose)

    pub.publish(pose_array)
    return pose_array


def pose_from_vector3D(vector):
    #http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    quat = Quaternion()
    #calculating the half-way vector.
    u = [1,0,0]
    norm = np.linalg.norm(vector)
    v = np.asarray(vector)/norm

    if (np.array_equal(u, v)):
        quat.w = 1
        quat.x = 0
        quat.y = 0
        quat.z = 0
    elif (np.array_equal(u, np.negative(v))):
        quat.w = 0
        quat.x = 0
        quat.y = 0
        quat.z = 1
    else:
        half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
        quat.w = np.dot(u, half)
        temp = np.cross(u, half)
        quat.x = temp[0]
        quat.y = temp[1]
        quat.z = temp[2]
    norm = np.math.sqrt(quat.x*quat.x + quat.y*quat.y + quat.z*quat.z + quat.w*quat.w)
    if norm == 0:
        norm = 1
    quat.x /= norm * -1
    quat.y /= norm * -1
    quat.z /= norm * -1
    quat.w /= norm * -1
    return quat

if __name__ == '__main__':
    
    rospy.init_node('find_poses_around_obj')
    s = rospy.Service('find_poses_around_obj', PosesAroundObj, get_poses)
    pub = rospy.Publisher('/debugger_poses_around_obj', PoseArray, queue_size=1, latch=True)

    print('Ready to find poses around object!')
    rospy.spin()

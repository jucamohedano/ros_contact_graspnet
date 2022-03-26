#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np
from math import atan2, cos, sin, sqrt, pi

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from pick_up_object.utils import detect_objs
from visualization_msgs.msg import Marker


marker = rospy.Publisher('/simple_contact', Marker, queue_size=30, latch=True)

def visualise_contact_pt(pose, id, frame_id, r=1.0, g=0., b=0):
    vis_pose = Marker()
    vis_pose.pose = pose
    vis_pose.header.frame_id = frame_id
    vis_pose.id = id
    vis_pose.type = Marker.SPHERE
    vis_pose.action = Marker.ADD
    vis_pose.scale.x = 0.03*1
    vis_pose.scale.y = 0.03*1
    vis_pose.scale.z = 0.03*1
    vis_pose.color.r = r
    vis_pose.color.g = g
    vis_pose.color.b = b
    vis_pose.color.a = 1.0
    vis_pose.lifetime = rospy.Duration()
    marker.publish(vis_pose)

def find_points(pcl_msg):
    rospy.init_node('find_points')
    """
    it finds the centroid, top left corner and right bottom corner
    """
    pcl = ros_numpy.numpify(pcl_msg)
    pcl = np.concatenate( (pcl['x'].reshape(-1,1), pcl['y'].reshape(-1,1), pcl['z'].reshape(-1,1)), axis=1)
    
    centroid = np.nanmean(pcl, axis=0)

    pose = Pose()
    pose.position.x = centroid[0]
    pose.position.y = centroid[1]
    pose.position.z = centroid[2]
    
    # visualise_contact_pt(pose, id=0, frame_id=pcl_msg.header.frame_id)

    top_left = np.nanmin(pcl, axis=0)
    print(top_left)    
    pose_top_left = Pose()
    pose_top_left.position.x = top_left[0]
    pose_top_left.position.y = top_left[1]
    pose_top_left.position.z = top_left[2]

    # visualise_contact_pt(pose_top_left, id=1, frame_id=pcl_msg.header.frame_id)

    bottom_right = np.nanmax(pcl, axis=0)
    print(bottom_right)    
    pose_bottom_right = Pose()
    pose_bottom_right.position.x = bottom_right[0]
    pose_bottom_right.position.y = bottom_right[1]
    pose_bottom_right.position.z = bottom_right[2]

    # visualise_contact_pt(pose_bottom_right, id=2, frame_id=pcl_msg.header.frame_id)

    find_mid_pts(centroid, top_left, bottom_right)

    # return pose, top_left, bottom_right

def find_mid_pts(centroid, top_l, bottom_r):
    x1 = top_l[0]
    x2 = bottom_r[0]
    mid_top_right = [(x2-x1)/2, top_l[1], top_l[2]]

    mid_top = Pose()
    mid_top.position.x = centroid[0]
    mid_top.position.y = top_l[1]
    mid_top.position.z = top_l[2]

    # visualise_contact_pt(mid_top, id=3, frame_id='xtion_rgb_optical_frame', r=0.,g=1.)

    mid_bottom = Pose()
    mid_bottom.position.x = centroid[0]
    mid_bottom.position.y = bottom_r[1]
    mid_bottom.position.z = bottom_r[2]

    # visualise_contact_pt(mid_bottom, id=4, frame_id='xtion_rgb_optical_frame', r=0.,g=1.)

    right_t = Pose()
    right_t.position.x = mid_top_right[0]
    right_t.position.y = mid_top_right[1]
    right_t.position.z = mid_top_right[2]

    # visualise_contact_pt(right_t, id=5, frame_id='xtion_rgb_optical_frame', r=0.,g=0.,b=1.0)

    height_mid_l = Pose()
    height_mid_l.position.x = top_l[0]
    height_mid_l.position.y = centroid[1]
    height_mid_l.position.z = centroid[2]

    visualise_contact_pt(height_mid_l, id=6, frame_id='xtion_rgb_optical_frame', r=0.,g=0.,b=1.0)

    height_mid_r = Pose()
    height_mid_r.position.x = bottom_r[0]
    height_mid_r.position.y = centroid[1]
    height_mid_r.position.z = centroid[2]

    visualise_contact_pt(height_mid_r, id=7, frame_id='xtion_rgb_optical_frame', r=0.,g=0.,b=1.0)

if __name__ == '__main__':
	# rospy.init_node('find_points')
    
    detection = detect_objs()
    find_points(detection.object_clouds[0])

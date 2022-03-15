# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

try:
    # Try Python 2.7 behaviour first
    from StringIO import StringIO
except ImportError:
    # Use Python 3.x behaviour as fallback and choose the non-unicode version
    from io import BytesIO as StringIO

from moveit_commander import MoveItCommanderException
from geometry_msgs.msg import Pose, PoseStamped, Transform
import rospy
import tf
# import open3d
from ctypes import * # convert float to uint32
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sensor_msgs.point_cloud2 as pc2


# open3d
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)

convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


def msg_to_string(msg):
    buf = StringIO()
    msg.serialize(buf)
    return buf.getvalue()


def msg_from_string(msg, data):
    msg.deserialize(data)


def pose_to_list(pose_msg):
    pose = []
    pose.append(pose_msg.position.x)
    pose.append(pose_msg.position.y)
    pose.append(pose_msg.position.z)
    pose.append(pose_msg.orientation.x)
    pose.append(pose_msg.orientation.y)
    pose.append(pose_msg.orientation.z)
    pose.append(pose_msg.orientation.w)
    return pose


def list_to_pose(pose_list):
    pose_msg = Pose()
    if len(pose_list) == 7:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]
    elif len(pose_list) == 6:
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        q = tf.transformations.quaternion_from_euler(
            pose_list[3], pose_list[4], pose_list[5]
        )
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
    else:
        raise MoveItCommanderException(
            "Expected either 6 or 7 elements in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)"
        )
    return pose_msg


def list_to_pose_stamped(pose_list, target_frame):
    pose_msg = PoseStamped()
    pose_msg.pose = list_to_pose(pose_list)
    pose_msg.header.frame_id = target_frame
    pose_msg.header.stamp = rospy.Time.now()
    return pose_msg


def transform_to_list(trf_msg):
    trf = []
    trf.append(trf_msg.translation.x)
    trf.append(trf_msg.translation.y)
    trf.append(trf_msg.translation.z)
    trf.append(trf_msg.rotation.x)
    trf.append(trf_msg.rotation.y)
    trf.append(trf_msg.rotation.z)
    trf.append(trf_msg.rotation.w)
    return trf


def list_to_transform(trf_list):
    trf_msg = Transform()
    trf_msg.translation.x = trf_list[0]
    trf_msg.translation.y = trf_list[1]
    trf_msg.translation.z = trf_list[2]
    trf_msg.rotation.x = trf_list[3]
    trf_msg.rotation.y = trf_list[4]
    trf_msg.rotation.z = trf_list[5]
    trf_msg.rotation.w = trf_list[6]
    return trf_msg

# def convertCloudFromRosToOpen3d(ros_cloud):
#     # Get cloud data from ros_cloud
#     field_names=[field.name for field in ros_cloud.fields]
#     print(field_names)
#     cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))
#     # Check empty
#     open3d_cloud = open3d.geometry.PointCloud()
#     if len(cloud_data)==0:
#         print("Converting an empty cloud")
#         return None

#     # Set open3d_cloud
#     if "rgb" in field_names:
#         IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
#         # Get xyz
#         xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

#         # Get rgb
#         # Check whether int or float
#         if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
#             rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
#         else:
#             rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

#         # combine
#         open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
#         open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
#     else:
#         xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
#         open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

#     # return
#     return open3d_cloud

# def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="xtion_depth_optical_frame"):
#     # Set "header"
#     header = Header()
#     header.stamp = rospy.Time.now()
#     header.frame_id = frame_id

#     # Set "fields" and "cloud_data"
#     points=np.asarray(open3d_cloud.points)
#     if not open3d_cloud.colors: # XYZ only
#         fields=FIELDS_XYZ
#         cloud_data=points
#     else: # XYZ + RGB
#         fields=FIELDS_XYZRGB
#         # -- Change rgb color from "three float" to "one 24-byte int"
#         # 0x00FFFFFF is white, 0x00000000 is black.
#         colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
#         colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
#         cloud_data=np.c_[points, colors]
    
#     # create ros_cloud
#     return pc2.create_cloud(header, fields, cloud_data)


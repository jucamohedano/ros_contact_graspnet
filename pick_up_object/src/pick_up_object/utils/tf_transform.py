#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs

from geometry_msgs.msg import PoseStamped, PoseArray
from pick_up_object.srv import TfTransform, TfTransformResponse

def tf_transform(msg):
    tf_response = TfTransformResponse()
    if msg.pose_array.header.frame_id != '':
        transformation = get_transform(source_frame=msg.pose_array.header.frame_id, target_frame=msg.target_frame.data)
        if transformation:
            pose_array = PoseArray()
            pose_array.header.frame_id = msg.target_frame.data
            pose_array.header.stamp = rospy.get_rostime()

            for pose in msg.pose_array.poses:
                new_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=pose), transformation).pose
                pose_array.poses.append(new_pose)

            tf_response.target_pose_array = pose_array
        else:
            print('Error: No transformation')
    if msg.pointcloud.header.frame_id != '':
        transformation = get_transform(source_frame=msg.pointcloud.header.frame_id, target_frame=msg.target_frame.data)
        if transformation:
            new_pointcloud = tf2_sensor_msgs.do_transform_cloud(msg.pointcloud, transformation)
            tf_response.target_pointcloud = new_pointcloud
        else:
            print('Error: No trasnformation')
   
    return tf_response

def get_transform(source_frame, target_frame):
    """
        Converts to target frame
        Returns the pose in the target frame
    """
    assert(source_frame and target_frame)
    # print('source_frame', source_frame)
    # print('target_frame', target_frame)
    try:
        transformation = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
        return transformation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)


if __name__ == '__main__':
    
    rospy.init_node('tf_transform_node')
    s = rospy.Service('tf_transform', TfTransform, tf_transform)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print('Ready to transform!')
    rospy.spin()

#!/usr/bin/env python3

import rospy
from geometric_grasp.srv import PosesAroundObj
from contact_graspnet.srv import DetectObjects
from contact_graspnet.utils import detect_objs, tf_transform

if __name__ == "__main__":
    rospy.init_node('test_reconstruction')

    resp_obj = detect_objs()
    obj = resp_obj.object_clouds[0]
    obj = tf_transform('base_footprint', pointcloud=obj).target_pointcloud

    rospy.wait_for_service('find_poses_around_obj', timeout=10)
    try:
        find_poses = rospy.ServiceProxy('find_poses_around_obj', PosesAroundObj)
        resp = find_poses(obj)
        print('we have the poses!')
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

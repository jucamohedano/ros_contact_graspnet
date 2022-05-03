#!/usr/bin/python

import rospy
from geometric_grasp.srv import PointcloudReconstructionRequest
from pick_up_object.srv import DetectObjects
from pick_up_object.utils import detect_objs

if __name__ == "__main__":
    rospy.init_node('test_reconstruction')

    resp_obj = detect_objs()
    obj = resp_obj.object_clouds[0]

    rospy.wait_for_service('pointcloud_reconstruction', timeout=10)
    try:
        detect = rospy.ServiceProxy('pointcloud_reconstruction', DetectObjects)
        resp = detect(obj)
        print('reconstruction done!')
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)    

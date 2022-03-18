import rospy
import numpy as np
from contact_graspnet.srv import GenerateGrasps, GenerateGraspsResponse, TfTransform, TfTransformRequest
# from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from contact_graspnet.srv import DetectObjects
from std_srvs.srv import Empty

def detect_objs():
    print('waiting for detect_objects')
    rospy.wait_for_service('detect_objects', timeout=10)
    try:
        detect = rospy.ServiceProxy('detect_objects', DetectObjects)
        resp = detect()
        print('detection done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def generate_grasps(full_pcl, objs_pcl):
    print('waiting for generate_grasps_server')
    rospy.wait_for_service('generate_grasps_server', timeout=10)
    try:
        grasp_poses = rospy.ServiceProxy('generate_grasps_server', GenerateGrasps)
        resp = grasp_poses(full_pcl, objs_pcl)
        print('generates grasps done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def clear_octomap():
    print('waiting for clear_octomap')
    rospy.wait_for_service('clear_octomap', timeout=10)
    try:
        clear_octo = rospy.ServiceProxy('clear_octomap', Empty)
        resp1 = clear_octo()
        print('clearing octomap done!')
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

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


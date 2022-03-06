import rospy
from contact_graspnet.srv import GenerateGraspsSrv, GenerateGraspsSrvResponse, TfTransform, TfTransformRequest
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from pick_up_object.srv import DetectObjects
from std_srvs.srv import Empty
import actionlib


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
        grasp_poses = rospy.ServiceProxy('generate_grasps_server', GenerateGraspsSrv)
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


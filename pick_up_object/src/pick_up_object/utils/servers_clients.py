import rospy
from contact_graspnet.srv import GenerateGrasps, GenerateGraspsResponse
from actionlib import SimpleActionClient


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

def generate_grasps(full_pcl):
    print('waiting for generate_grasps_server')
    rospy.wait_for_service('generate_grasps_server', timeout=10)
    try:
        grasp_poses = rospy.ServiceProxy('generate_grasps_server', GenerateGrasps)
        resp = grasp_poses(full_pcl)
        print('generates grasps done!')
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

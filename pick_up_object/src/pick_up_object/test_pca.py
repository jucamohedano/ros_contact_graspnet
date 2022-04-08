#!/usr/bin/env python
import rospy
from pick_up_object.utils import detect_objs
from sensor_msgs.msg import PointCloud2
from pcl_manipulation.srv import PCA

if __name__ == "__main__":
    rospy.init_node('publish_cloud_obj')
    # pub = rospy.Publisher('/save_to_pcd', PointCloud2, queue_size=1, latch=True)
    det_resp = detect_objs()
    # pub.publish(det_resp.object_clouds[0])
    
    
    rospy.wait_for_service('/pcl_manipulation/pca_basis', timeout=10)
    try:
        detect = rospy.ServiceProxy('/pcl_manipulation/pca_basis', PCA)
        resp = detect(det_resp.object_clouds[0])
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    print(resp)


    
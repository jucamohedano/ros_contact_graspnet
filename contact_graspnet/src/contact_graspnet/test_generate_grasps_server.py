#!/usr/bin/python3

import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
from contact_graspnet.srv import GenerateGraspsSrv


if __name__ == '__main__':
    rospy.init_node('test_generate_grasps_server')

    pcl_msg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

    pcl = ros_numpy.numpify(pcl_msg)
    pcl = np.concatenate( (pcl['x'].reshape(-1,1), pcl['y'].reshape(-1,1), pcl['z'].reshape(-1,1)), axis=1)

    # uncomment to save point cloud to file
    # with open('tiago_pcl.npy', 'wb') as f:
    #     np.save(f, pcl)

    rospy.wait_for_service('generate_grasps_service')
    try:
        generate_grasps_service = rospy.ServiceProxy('generate_grasps_service', GenerateGraspsSrv)
        resp1 = generate_grasps_service(pcl_msg)


        print(resp1.result)
        print("nice!")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
import rospy
from contact_graspnet.srv import TfTransform, TfTransformRequest

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
#!/usr/bin/python
import smach
import rospy
from pick_up_object.utils import clear_octomap
from geometry_msgs.msg import PoseArray

class ApproachObject(smach.State):
    
    def __init__(self, arm_torso_controller, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'grasps_resp', 'collision_objs', 'object_index'],
                             output_keys=['grasps_resp', 'prev', 'collision_objs', 'object_index'])
        self.arm_torso = arm_torso_controller
        self.planning_scene = planning_scene
        self.target_poses_pub = rospy.Publisher('/current_target_poses', PoseArray, queue_size=100, latch=True)
        self.num_objects = 0
        
        self.retry_attempts = 3
        self.try_num = 0

    def execute(self, userdata):
        
        userdata.prev = 'ApproachObject'
        
        grasps_poses = userdata.grasps_resp
        object_index = userdata.object_index
        # object_index = 0

        rospy.loginfo("Approaching object_{}".format(object_index))
        # rospy.loginfo("{} grasps for object_{}".format(len(grasps_poses[object_index].poses), object_index))

        
        # grasps_poses.all_grasp_poses[0].header.frame_id='gripper_grasping_frame'
        # print(grasps_poses.all_grasp_poses[0])
        
        # self.arm_torso.update_planning_scene(add=True)
        # self.target_poses_pub.publish(grasps_poses[object_index])
        # self.arm_torso.configure_planner()
        if not grasps_poses[0].poses:
            rospy.loginfo('There are no grasps for object_{}'.format(object_index))
            return 'failed'
        result = self.arm_torso.sync_reach_ee_poses(grasps_poses[0])
        clear_octomap()

        if not result:
            result='failed'
        else:
            result = 'succeeded'

        return result
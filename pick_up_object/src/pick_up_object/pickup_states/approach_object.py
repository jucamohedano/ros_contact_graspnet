#!/usr/bin/python
import smach
import rospy
from pick_up_object.utils import clear_octomap
from geometry_msgs.msg import PoseArray

class ApproachObject(smach.State):
    
    def __init__(self, arm_torso_controller, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'grasps_resp', 'collision_obj'],
                             output_keys=['grasps_resp', 'prev', 'collision_obj'])
        self.arm_torso = arm_torso_controller
        self.planning_scene = planning_scene
        self.target_poses_pub = rospy.Publisher('/current_target_poses', PoseArray, queue_size=70, latch=True)

        
        self.retry_attempts = 3
        self.try_num = 0

    def execute(self, userdata):
        
        userdata.prev = 'ApproachObject'
        
        grasps_poses = userdata.grasps_resp

        clear_octomap()
        rospy.sleep(0.5)
        # grasps_poses.all_grasp_poses[0].header.frame_id='gripper_grasping_frame'
        # print(grasps_poses.all_grasp_poses[0])
        
        self.target_poses_pub.publish(grasps_poses.all_grasp_poses[0])
        self.arm_torso.configure_planner()
        result = self.arm_torso.sync_reach_ee_poses(grasps_poses.all_grasp_poses[0])
        
        # add add object to collision matrix of planning scene
        # self.arm_torso.update_planning_scene(add=True)

        if not result:
            print('Approach Failed')
            self.arm_torso.sync_reach_safe_joint_space()
            result='failed'
        else:
            result = 'succeeded'

        return result
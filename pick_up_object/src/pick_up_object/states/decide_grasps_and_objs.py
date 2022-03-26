#!/usr/bin/python

import rospy
import smach
# import dynamic_reconfigure.client

from pick_up_object.utils import add_collision_object, clear_octomap, tf_transform

# TODO: create a substate machine with the code from pick_object function

class DecideGraspsAndObjs(smach.State):
    
    def __init__(self, arm_torso_controller):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['objs_resp', 'grasps_resp'],
                             output_keys=['prev', 'collision_obj']
                             )
        self.planning_scene = arm_torso_controller._scene
        self.arm_torso = arm_torso_controller

    def execute(self, userdata):
        userdata.prev = 'DecideGraspsAndObjs'
        objs_resp = userdata.objs_resp
        
        eef_link = self.arm_torso.move_group.get_end_effector_link()

        # remove any previous objects added to the planning scene
        if self.planning_scene.get_attached_objects(object_ids=['object']):
            self.planning_scene.remove_attached_object(eef_link, name='object')
        if self.planning_scene.get_objects(object_ids=['object']):
            self.planning_scene.remove_world_object('object')

        # result = self.pick_object(objs_resp, grasps_resp.all_grasp_poses[0], 0)

        # cloud = tf_transform(target_frame='base_footprint', pointcloud=objs_resp.object_clouds[0]).target_pose_array

        # hard coding the index of the object to pick up
        co = add_collision_object(objs_resp.object_clouds[0], self.planning_scene)
        rospy.sleep(1.)
        userdata.collision_obj = co

        return 'succeeded'
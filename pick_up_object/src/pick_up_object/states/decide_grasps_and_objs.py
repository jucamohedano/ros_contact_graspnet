#!/usr/bin/python

import rospy
import smach
# import dynamic_reconfigure.client

from pick_up_object.utils import add_collision_object, clear_octomap, play_motion_action

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
        play_motion_action('home')
        userdata.prev = 'DecideGraspsAndObjs'
        objs_resp = userdata.objs_resp
        
        self.arm_torso.configure_planner()
        eef_link = self.arm_torso.move_group.get_end_effector_link()

        # remove any previous objects added to the planning scene
        if self.planning_scene.get_attached_objects(object_ids=['object']):
            self.planning_scene.remove_attached_object(eef_link, name='object')
        if self.planning_scene.get_objects(object_ids=['object']):
            self.planning_scene.remove_world_object('object')

        # hard coding the index of the object to pick up
        rospy.loginfo("Adding collision object with id='object' to the planning scene")
        co = add_collision_object(objs_resp.object_clouds[0], self.planning_scene, num_primitives=50)
        clear_octomap()
        rospy.sleep(1.)
        userdata.collision_obj = co

        return 'succeeded'
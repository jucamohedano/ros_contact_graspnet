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
                             input_keys=['objs_resp', 'grasps_resp', 'euclidean_clusters'],
                             output_keys=['prev']
                             )
        self.planning_scene = arm_torso_controller._scene
        self.arm_torso = arm_torso_controller

    def execute(self, userdata):
        # play_motion_action('home')
        userdata.prev = 'DecideGraspsAndObjs'
        objs_resp = userdata.objs_resp
        # euclidean_clusters = userdata.euclidean_clusters
        
        self.arm_torso.configure_planner()
        eef_link = self.arm_torso.move_group.get_end_effector_link()

        # remove any previous objects added to the planning scene
        if self.planning_scene.get_attached_objects():
            self.planning_scene.remove_attached_object()
        if self.planning_scene.get_objects():
            # self.planning_scene.remove_world_object('object')
            self.planning_scene.clear()

        # hard coding the index of the object to pick up
        rospy.loginfo("Adding collision object with id='object' to the planning scene")

        print('len(objs_resp.clusters)', len(objs_resp.clusters))
        co = [add_collision_object(cluster, self.planning_scene, num_primitives=50, id='object_{}'.format(i)) for i, cluster in enumerate(objs_resp.clusters)]
        
        clear_octomap()
        rospy.sleep(3.)
        # userdata.collision_obj = co

        return 'succeeded'
#!/usr/bin/python

import rospy
import smach
# import dynamic_reconfigure.client

from pick_up_object.utils import add_collision_object, clear_octomap, play_motion_action

class DecideGraspsAndObjs(smach.State):
    
    def __init__(self, arm_torso_controller):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'looping'],
                             input_keys=['prev', 'objs_resp', 'grasps_resp'],
                             output_keys=['prev', 'collision_objs', 'object_index']
                             )
        self.planning_scene = arm_torso_controller._scene
        self.arm_torso = arm_torso_controller
        self.object_index = 0
        config = dict()
        config['planning_attempts'] = 10
        config['planning_time'] = 0.5
        config['num_planning_attempts'] = 10
        config['goal_joint_tolerance'] = 0.003
        config['goal_pos_tol'] = 0.001
        config['goal_orien_tol'] = 0.001
        self.arm_torso.configure_planner(config)

    def execute(self, userdata):
	if userdata.prev == 'GenerateGrasps':
	    self.object_index = 0
        if userdata.prev == 'Recovery' or userdata.prev == 'Dropoff':
            self.object_index += 1
            if self.object_index >= (len(userdata.objs_resp.object_clouds)-1):
                userdata.prev = 'DecideGraspsAndObjs'
                return 'failed'

        userdata.prev = 'DecideGraspsAndObjs'
        objs_resp = userdata.objs_resp
        grasps_poses = userdata.grasps_resp
        
        
        eef_link = self.arm_torso.move_group.get_end_effector_link()

        # remove any previous objects added to the planning scene
        if self.planning_scene.get_attached_objects():
            self.planning_scene.remove_attached_object()
        if self.planning_scene.get_objects():
            # self.planning_scene.remove_world_object('object')
            self.planning_scene.clear()

        # hard coding the index of the object to pick up
        rospy.loginfo("Adding collision object_{}={} to the planning scene".format(self.object_index, objs_resp.labels_text[self.object_index].data))

        # co = [add_collision_object(cluster, self.planning_scene, num_primitives=50, id='object_{}'.format(i)) for i, cluster in enumerate(objs_resp.clusters)]
        # while not grasps_poses[self.object_index].poses:
        #     rospy.loginfo('There are no grasps for object_{}={}'.format(self.object_index, objs_resp.labels_text[self.object_index].data))
        #     self.object_index += 1
        #     return 'looping'
        self.remove_empty_grasps(grasps_poses, objs_resp)
        
        # collision_objs = [
        #     add_collision_object(cluster, self.planning_scene, num_primitives=100, id='object_{}'.format(i))
        #                 for i, cluster in enumerate(objs_resp.object_clouds)]
        collision_objs = [add_collision_object(objs_resp.object_clouds[self.object_index], self.planning_scene, num_primitives=100, id='object_{}'.format(self.object_index))]

        


        clear_octomap()
        rospy.sleep(0.5)
        userdata.collision_objs = collision_objs
        userdata.object_index = self.object_index

        return 'succeeded'


    def remove_empty_grasps(self, grasps, objects):
        for i, g in enumerate(grasps):
            if not g:
                rospy.loginfo('Removing: {} grasps for object_{}'.format(g, i))
                del grasps[i]
                del objects[i]

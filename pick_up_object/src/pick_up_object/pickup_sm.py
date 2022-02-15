#!/usr/bin/env python
import rospy
import smach
from pick_up_object.controllers import *
from pick_up_object.states import GenerateGrasps, Pickup, DropOff, Recovery
from pick_up_object.utils.planning_scene_interface import PlanningSceneInterface

# TODO: pick up state machine


if __name__ == '__main__':
    rospy.init_node('pick_up_sm', anonymous=True)

    arm_torso_controller = ArmTorsoController()
    planning_scene = PlanningSceneInterface()


    sm = smach.StateMachine(outcomes=['succeeded','failed', 'end'])

    smach.StateMachine.add('DetectObjects', DetectObjects(head_controller, torso_controller),
                            transitions={
                                'succeeded': 'GenerateGrasps',
                                'looping': 'DetectObjects',
                                'failed': 'Recovery'},
                            remapping={
                                'prev': 'prev',
                                'objs_resp': 'objs_resp'})
    smach.StateMachine.add('GenerateGrasps', GenerateGrasps(),
                            transitions={
                                'succeeded': 'Pickup',
                                'failed': 'Recovery'}, 
                            remapping={
                                'grasps_resp': 'grasps_resp',
                                'objs_resp': 'objs_resp'
                            })
    smach.StateMachine.add('Pickup', Pickup(planning_scene, arm_torso_controller),
                            transitions={
                                'succeeded': 'Dropoff',
                                'failed': 'Recovery'}, 
                            remapping={})
    smach.StateMachine.add('Dropoff', DropOff(),
                            transitions={
                                'succeeded': 'Finish',
                                'failed': 'Finish'}, 
                            remapping={})
    smach.StateMachine.add('Recovery', Recovery(),
                            transitions={
                                'succeeded' : 'Finish',
                                'failed': 'Finish'},
                            remapping={})
    smach.StateMachine.add('Finish', Finish(),
                            transitions={
                                'end' : 'end'},
                            remapping={})
    
    sm.set_initial_state(['GenerateGrasps'], userdata=smach.UserData())

    rospy.spin()

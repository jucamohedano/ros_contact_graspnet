#!/usr/bin/env python
import rospy
import smach
from pick_up_object.controllers import *
from pick_up_object.states import GenerateGrasps, Pickup, Recovery, DetectObjects

# TODO: update states transitions
# TODO: dropoff state

if __name__ == '__main__':
    rospy.init_node('pickup_sm', anonymous=True)

    arm_torso_controller = ArmTorsoController()
    head_controller = HeadController()
    torso_controller = TorsoController()

    sm = smach.StateMachine(outcomes=['succeeded','failed', 'end'])
    sm.userdata.prev = 'start'

    with sm:
        smach.StateMachine.add('DetectObjects', DetectObjects(head_controller, torso_controller, arm_torso_controller),
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
                                    'looping' : 'DetectObjects',
                                    'failed': 'Recovery'}, 
                                remapping={
                                    'prev' : 'prev',
                                    'objs_resp' : 'objs_resp',
                                    'grasps_resp': 'grasps_resp'})
        smach.StateMachine.add('Pickup', Pickup(arm_torso_controller),
                                transitions={
                                    'succeeded': 'end',
                                    'failed': 'Recovery'},
                                remapping={
                                    'prev': 'prev',
                                    'objs_resp': 'objs_resp',
                                    'grasps_resp': 'grasps_resp'})
        # smach.StateMachine.add('Dropoff', DropOff(),
        #                         transitions={
        #                             'succeeded': 'Finish',
        #                             'failed': 'Finish'}, 
        #                         remapping={})
        smach.StateMachine.add('Recovery', Recovery(),
                                transitions={
                                    'failed': 'end', 
                                    'restart' : 'DetectObjects', 
                                    'restart_object_detection' : 'DetectObjects'},
                                remapping={
                                    'prev' : 'prev'
                                })
    sm.set_initial_state(['DetectObjects'], userdata=smach.UserData())
    sm.execute()
    rospy.spin()

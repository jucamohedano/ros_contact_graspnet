#!/usr/bin/env python
import rospy
import smach
import smach_ros
from pick_up_object.controllers import *
from pick_up_object.states import GenerateGrasps, Pickup, Recovery, DetectObjects, DecideGraspsAndObjs, GenerateGeometricGrasps
from pick_up_object.pickup_object_sm import PickupObject_SM

# TODO: update states transitions
# TODO: dropoff state

if __name__ == '__main__':
    rospy.init_node('pickup_sm', anonymous=True)

    arm_torso_controller = ArmTorsoController()
    gripper_controller = GripperController()
    head_controller = HeadController()
    torso_controller = TorsoController()
    pickup_object_sm = PickupObject_SM(arm_torso_controller, gripper_controller)


    sm = smach.StateMachine(outcomes=['succeeded','failed', 'end'])
    sm.userdata.prev = 'start'
    sis = smach_ros.IntrospectionServer('pickup_sm', sm, '/SM_PICKUP_TOP')
    sis.start()

    with sm:
        smach.StateMachine.add('DetectObjects', DetectObjects(head_controller, torso_controller, arm_torso_controller),
                                transitions={
                                    'succeeded': 'GenerateGeometricGrasps',
                                    'looping': 'DetectObjects',
                                    'failed': 'Recovery'},
                                remapping={
                                    'prev': 'prev',
                                    'objs_resp': 'objs_resp',
                                    })
        smach.StateMachine.add('GenerateGrasps', GenerateGrasps(),
                                transitions={
                                    'succeeded': 'DecideGraspsAndObjs',
                                    'looping' : 'DetectObjects',
                                    'failed': 'Recovery'}, 
                                remapping={
                                    'prev' : 'prev',
                                    'objs_resp' : 'objs_resp',
                                    'grasps_resp': 'grasps_resp',
                                    })
        
        smach.StateMachine.add('GenerateGeometricGrasps', GenerateGeometricGrasps(arm_torso_controller),
                                transitions={
                                    'succeeded': 'DecideGraspsAndObjs',
                                    'looping' : 'DetectObjects',
                                    'failed': 'Recovery'}, 
                                remapping={
                                    'prev' : 'prev',
                                    'objs_resp' : 'objs_resp',
                                    'grasps_resp': 'grasps_resp',
                                    })

        smach.StateMachine.add('DecideGraspsAndObjs', DecideGraspsAndObjs(arm_torso_controller),
                                transitions={
                                    'succeeded': 'Pickup',
                                    'failed': 'end'}, 
                                remapping={
                                    'prev' : 'prev',
                                    'objs_resp' : 'objs_resp',
                                    'grasps_resp': 'grasps_resp',
                                    'collision_obj' : 'collision_obj',
                                    })
        # sub state machine
        pickup_object_sm.add_states()
        smach.StateMachine.add('Pickup', pickup_object_sm.sm, 
                                transitions={'success' : 'end',
                                            'failed' : 'DetectObjects'
                                            },
                                remapping={
                                    'prev' : 'prev',
                                    'objs_resp' : 'objs_resp',
                                    'grasps_resp': 'grasps_resp',
                                    'collision_obj' : 'collision_obj'})

        smach.StateMachine.add('Recovery', Recovery(),
                                transitions={
                                    'failed': 'end', 
                                    'restart' : 'DetectObjects', 
                                    'restart_object_detection' : 'DetectObjects'},
                                remapping={
                                    'prev' : 'prev'
                                })
        
        
        
        
        # smach.StateMachine.add('Pickup', Pickup(arm_torso_controller),
        #                         transitions={
        #                             'succeeded': 'end',
        #                             'failed': 'Recovery'},
        #                         remapping={
        #                             'prev': 'prev',
        #                             # 'objs_resp': 'objs_resp',
        #                             'grasps_resp': 'grasps_resp'})
        # smach.StateMachine.add('Dropoff', DropOff(),
        #                         transitions={
        #                             'succeeded': 'Finish',
        #                             'failed': 'Finish'}, 
        #                         remapping={})
        
    sm.set_initial_state(['DetectObjects'], userdata=smach.UserData())
    sm.execute()
    rospy.spin()

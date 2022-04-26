#!/usr/bin/env python
import rospy
import smach
import smach_ros
from pick_up_object.controllers import *
from pick_up_object.states import GenerateGrasps, Pickup, Recovery, DetectObjects, DecideGraspsAndObjs, GenerateGeometricGrasps, Dropoff, GoToTable
from pick_up_object.pickup_object_sm import PickupObject_SM
import time

if __name__ == '__main__':
    rospy.init_node('pickup_sm', anonymous=True)

    arm_torso_controller = ArmTorsoController()
    arm_torso_controller._scene.clear()
    gripper_controller = GripperController()
    head_controller = HeadController()
    torso_controller = TorsoController()
    base_controller = BaseController()
    pickup_object_sm = PickupObject_SM(arm_torso_controller, gripper_controller)
    startTime = time.time()

    sm = smach.StateMachine(outcomes=['succeeded','failed', 'end'])
    sm.userdata.prev = 'start'
    sis = smach_ros.IntrospectionServer('pickup_sm', sm, '/SM_PICKUP_TOP')
    sis.start()

    with sm:
        smach.StateMachine.add('GoToTable', GoToTable(head_controller, base_controller, arm_torso_controller),
                                transitions={
                                    'succeeded': 'DetectObjects',
                                    'failed': 'Recovery'},
                                remapping={
                                    'prev': 'prev',
                                    })
        smach.StateMachine.add('DetectObjects', DetectObjects(head_controller, torso_controller, arm_torso_controller),
                                transitions={
                                    'succeeded': 'GenerateGrasps',
                                    'looping': 'DetectObjects',
                                    'failed': 'Recovery'},
                                remapping={
                                    'prev': 'prev',
                                    'objs_resp': 'objs_resp',
                                    })
        smach.StateMachine.add('GenerateGrasps', GenerateGrasps(arm_torso_controller, '_geometricGrasping'),
                                transitions={
                                    'succeeded': 'DecideGraspsAndObjs',
                                    'looping' : 'DetectObjects',
                                    'failed': 'Recovery'}, 
                                remapping={
                                    'prev' : 'prev',
                                    'objs_resp' : 'objs_resp',
                                    'grasps_resp': 'grasps_resp',
                                    'object_index' : 'object_index'
                                    })
        
        # smach.StateMachine.add('GenerateGeometricGrasps', GenerateGeometricGrasps(arm_torso_controller),
        #                         transitions={
        #                             'succeeded': 'DecideGraspsAndObjs',
        #                             'looping' : 'DetectObjects',
        #                             'failed': 'Recovery'}, 
        #                         remapping={
        #                             'prev' : 'prev',
        #                             'objs_resp' : 'objs_resp',
        #                             'grasps_resp': 'grasps_resp',
        #                             })

        smach.StateMachine.add('DecideGraspsAndObjs', DecideGraspsAndObjs(arm_torso_controller),
                                transitions={
                                    'succeeded': 'Pickup',
                                    'failed': 'Recovery',
                                    'looping' : 'DecideGraspsAndObjs'}, 
                                remapping={
                                    'prev' : 'prev',
                                    'objs_resp' : 'objs_resp',
                                    'grasps_resp': 'grasps_resp',
                                    'collision_objs' : 'collision_objs',
                                    'object_index' : 'object_index'
                                    })
        # sub state machine
        pickup_object_sm.add_states()
        smach.StateMachine.add('Pickup', pickup_object_sm.sm, 
                                transitions={'success' : 'Dropoff',
                                            'failed' : 'Recovery'
                                            },
                                remapping={
                                    'prev' : 'prev',
                                    'objs_resp' : 'objs_resp',
                                    'grasps_resp': 'grasps_resp',
                                    'collision_objs' : 'collision_objs',
                                    'object_index' : 'object_index'
                                    })

        smach.StateMachine.add('Recovery', Recovery(),
                                transitions={
                                    'failed': 'end', 
                                    'restart' : 'GoToTable',
                                    'change_grasps' : 'DecideGraspsAndObjs'
                                    },
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
        smach.StateMachine.add('Dropoff', Dropoff(base_controller, arm_torso_controller, startTime),
                                transitions={
                                    'succeeded': 'GoToTable',
                                    'failed': 'end'}, 
                                remapping={
                                    'prev' : 'prev',
                                    'collision_objs' : 'collision_objs'
                                })
        
    sm.set_initial_state(['GoToTable'], userdata=smach.UserData())
    sm.execute()
    rospy.spin()

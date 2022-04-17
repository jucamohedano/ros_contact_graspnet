#!/usr/bin/env python
import smach
import smach_ros
from pick_up_object.pickup_states import *

class PickupObject_SM():

    def __init__(self, arm_torso_controller, gripper_controller):
        self.arm_torso = arm_torso_controller
        self.gripper = gripper_controller
        self.planning_scene = self.arm_torso._scene

        self.sm = smach.StateMachine(input_keys=['prev', 'objs_resp', 'grasps_resp', 'collision_objs', 'object_index'],
                                    output_keys=['prev'],
                                    outcomes=['success','failed'])
        sis = smach_ros.IntrospectionServer('pickup_sm', self.sm, '/SM_PICKUP_TOP/SM_PICKUP_SUB')
        sis.start()

        

    def add_states(self):
        
        with self.sm:
            smach.StateMachine.add('ApproachObject', ApproachObject(self.arm_torso, self.planning_scene),
                                    transitions={
                                        'succeeded': 'ShiftForward',
                                        'failed': 'PickupRecovery'},
                                    remapping={
                                        'prev': 'prev',
                                        'grasps_resp' : 'grasps_resp',
                                        'collision_objs' : 'collision_objs',
                                        'object_index' : 'object_index'
                                        })
            smach.StateMachine.add('ShiftForward', ShiftForward(self.arm_torso, self.gripper, self.planning_scene),
                                    transitions={
                                        'succeeded': 'GraspObject',
                                        'failed': 'PickupRecovery'},
                                    remapping={
                                        'prev' : 'prev',
                                        'collision_objs' : 'collision_objs',
                                        'object_index' : 'object_index'
                                        })
            smach.StateMachine.add('GraspObject', GraspObject(self.arm_torso, self.gripper, self.planning_scene),
                                    transitions={
                                        'succeeded': 'LiftObject',
                                        'failed': 'PickupRecovery'},
                                    remapping={
                                        'prev': 'prev',
                                        'grasps_resp': 'grasps_resp',
                                        'collision_objs' : 'collision_objs',
                                        'object_index' : 'object_index'
                                        })
            smach.StateMachine.add('LiftObject', LiftObject(self.arm_torso, self.planning_scene),
                                    transitions={
                                        'succeeded': 'success',
                                        'failed': 'PickupRecovery'},
                                    remapping={
                                        'prev': 'prev',
                                        'collision_objs' : 'collision_objs'
                                        })
            smach.StateMachine.add('PickupRecovery', PickupRecovery(self.arm_torso, self.gripper, self.planning_scene),
                                    transitions={
                                        'failed': 'failed', 
                                        },
                                    remapping={
                                        'prev' : 'prev',
                                        'collision_objs' : 'collision_objs'
                                    })

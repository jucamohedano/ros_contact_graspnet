#!/usr/bin/env python
import rospy
import smach


# TODO: pick up state machine


if __name__ == '__main__':
    rospy.init_node('pick_up_sm', anonymous=True)

    sm = smach.StateMachine(outcomes=['succeeded','failed', 'end'])

    smach.StateMachine.add('GenerateGrasps', GenerateGrasps(),
                            transitions={
                                'succeeded': 'Pickup',
                                'failed': 'Recovery'}, 
                            remapping={})
    smach.StateMachine.add('Pickup', Pickup(),
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
    
    sm.set_initial_state(['Pickup'], userdata=smach.UserData())

    rospy.spin()

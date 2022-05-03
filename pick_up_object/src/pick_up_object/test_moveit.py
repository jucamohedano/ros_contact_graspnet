#!/usr/bin/env python
import rospy
from pick_up_object.controllers import ArmTorsoController
import moveit_commander
import time

def load_move_group(name, wfs=0):
    st = time.time()
    mg = None
    while not mg:
        try:
            mg = moveit_commander.MoveGroupCommander(name)
        except RuntimeError as re:
            print(re.message)
            rospy.sleep(1)
    rospy.loginfo(": [" + name.upper() + "] Time taken to load move group: " +
        "{:.2f}".format(time.time() - st) + " seconds.")
    return mg


if __name__ == '__main__':
    rospy.init_node('test_arm_move')

    arm_torso = ArmTorsoController()
    #arm_torso.configure_planner()

    arm_torso.sync_reach_safe_joint_space()
    #load_move_group('arm_torso', wfs=15)

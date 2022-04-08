#!/usr/bin/python
import smach
import rospy

from pick_up_object.utils import clear_octomap

class ShiftForward(smach.State):
    
    def __init__(self, arm_torso_controller, gripper, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'collision_obj'],
                             output_keys=['prev', 'collision_obj'])
        
        self.gripper = gripper
        self.arm_torso = arm_torso_controller
        self.planning_scene = planning_scene
        
        self.retry_attempts = 3
        self.try_num = 0

    def execute(self, userdata):
        
        userdata.prev = 'ShiftForward'
        
        print('About to Shift Forward')

        # self.planning_scene.remove_world_object('object')
        # self.arm_torso.update_planning_scene(add=True)


        # moveit planner params
        # config = dict()
        # config['planning_attempts'] = 1
        # config['planning_time'] = 5.
        # config['num_planning_attempts'] = 5
        # self.arm_torso.configure_planner(config)

        # check that gripper is open
        gripper_state = self.gripper.gripper_state()
        if gripper_state.position[0] < 0.3:
            self.gripper.sync_reach_to(self.gripper.JOINT_MAX)

        # move forward gripper in gripper_grasping_frame
        shift = 0.2
        clear_octomap()
        rospy.sleep(1.)
        result = self.arm_torso.sync_shift_ee(x=shift)
        print(result)
        # for i in range(3):
            # print('Shift Try: {}'.format(i + 1))
            # result = self.arm_torso.sync_shift_ee(x=shift)
            # if result:
            #     break
            # else:
            #     shift -= 0.03

        # reset planning configuration
        # del config['planning_attempts']
        # self.arm_torso.configure_planner(config)

        rospy.sleep(1.)

        if not result:
            print('Shift Forward Failed')
            result='failed'

            self.arm_torso.sync_reach_safe_joint_space()
            self.planning_scene.remove_world_object('object')
        else:
            result = 'succeeded'


        return result
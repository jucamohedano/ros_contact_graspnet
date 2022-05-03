#!/usr/bin/python
import smach
import rospy

from pick_up_object.utils import clear_octomap
from geometry_msgs.msg import WrenchStamped

class ShiftForward(smach.State):
    
    def __init__(self, arm_torso_controller, gripper, planning_scene):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'collision_objs', 'object_index'],
                             output_keys=['prev', 'collision_objs', 'object_index'])
        
        self.gripper = gripper
        self.arm_torso = arm_torso_controller
        self.planning_scene = planning_scene
        self.ft_sub = rospy.Subscriber('/wrist_ft', WrenchStamped, self.force_torque_cb, queue_size=1)
        
        self.ft_sensor = 0
        self.retry_attempts = 3
        self.try_num = 0

    def execute(self, userdata):
        
        userdata.prev = 'ShiftForward'
        

        # self.planning_scene.remove_world_object('object')
        # objs = self.planning_scene.get_objects()
        # objs_ids = [obj_id for obj_id in objs.id]
        print('planning_scene collision objects {}'.format(self.planning_scene.get_known_object_names()))
        # self.arm_torso.update_planning_scene(add=True, entry_names=['object_0'])
        self.arm_torso.update_planning_scene(add=True, entry_names=self.planning_scene.get_known_object_names())

        

        # check that gripper is open
        gripper_state = self.gripper.gripper_state()
        if gripper_state.position[0] < 0.3:
            self.gripper.sync_reach_to(self.gripper.JOINT_MAX)

        # move forward gripper in gripper_grasping_frame
        shift = 0.165
        clear_octomap()

        # config = dict()
        # config['planning_time'] = 0.5
        # config['goal_joint_tolerance'] = 0.01
        # config['num_planning_attempts'] = 10
        # config['goal_pos_tol'] = 0.01
        # config['goal_orien_tol'] = 0.01
        # config['max_acceleration'] = 0.1
        # config['max_velocity'] = 0.1
        # self.arm_torso.configure_planner(config)
        # self.arm_torso._move_group.clear_path_constraints()
        # rospy.sleep(3.5)

        result = self.arm_torso.sync_shift_ee(x=shift, _wait=True)
        if self.ft_sensor == 1:
            rospy.loginfo('hitting the table but returning True!')
            result = True
            self.ft_sensor = 0
        

        # config['goal_joint_tolerance'] = 0.003
        # config['goal_pos_tol'] = 0.001
        # config['goal_orien_tol'] = 0.001
        # self.arm_torso.configure_planner(config)

        rospy.loginfo('shift forward result {}'.format(result))


        if not result:
            result='failed'
            self.arm_torso.sync_reach_safe_joint_space()
            # self.planning_scene.remove_world_object('object')
            self.planning_scene.clear()
        else:
            result = 'succeeded'


        return result

    def force_torque_cb(self, msg):
        f = msg.wrench.force
        if f.z < -8:
            self.ft_sensor = 1
            rospy.loginfo('Hitting the table, stop shift forward goal')  
            self.arm_torso._move_group.stop()

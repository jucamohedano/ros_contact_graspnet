#!/usr/bin/env python

import rospy
import smach
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import time

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from pick_up_object.utils import play_motion_action, clear_octomap
from std_msgs.msg import Header

class Dropoff(smach.State):
    
    def __init__(self, base_controller, arm_torso_controller, startTime):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['prev', 'collision_objs'],
                             output_keys=['prev']
                             )
        self.startTime = startTime
        self.arm_torso = arm_torso_controller
        self.base = base_controller
        self.planning_scene = arm_torso_controller._scene
        self.bin = rospy.get_param('/base_green_bin')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
	    # self.startTime = startTime


    def execute(self, userdata):
        userdata.prev = 'Dropoff'

        config = dict()
        config['goal_joint_tolerance'] = 0.01
        config['goal_pos_tol'] = 0.01
        config['goal_orien_tol'] = 0.01
        self.arm_torso.configure_planner(config)


        p = [self.bin['position']['x'], self.bin['position']['y'], self.bin['position']['z']]
        o = [self.bin['orientation']['x'], self.bin['orientation']['y'], self.bin['orientation']['z'], self.bin['orientation']['w']]
        result = self.base.sync_reach_to(position=p, quaternion=o)
        clear_octomap()
        if result:
            try:
                trans = self.tfBuffer.lookup_transform('base_footprint', 'map', rospy.Time(0), rospy.Duration(0.1))
                
                header = Header(seq=1, stamp=rospy.Time.now(), frame_id='base_footprint')
                p = PoseStamped(
                            header=header,
                            pose=Pose(
                                Point(p[0], p[1], p[2]),
                                Quaternion(o[0], o[1], o[2], o[3])
                                )
                            )
                p = tf2_geometry_msgs.do_transform_pose(p, trans).pose
                result = self.arm_torso.sync_reach_ee_pose(
                    (p.position.x+0.55, p.position.y, p.position.z+1.),
                    (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                )
                if result:
                    play_motion_action('open')
                    aobjs = self.planning_scene.get_attached_objects()
                    rospy.loginfo("Objects attached are {}".format(aobjs.keys()))
                    if aobjs:
                        rospy.loginfo("Removing object_{}".format(aobjs.keys()[0]))
                        self.planning_scene.remove_attached_object(aobjs.keys()[0])
                        rospy.sleep(1.)
                        
                    executionTime = (time.time() - self.startTime)
                    with open('full_runs_timings.txt', 'a+') as f:
                        f.write('Full run state machine execution time: ' + str(executionTime) + '\n')
                    
                    config['goal_joint_tolerance'] = 0.003
                    config['goal_pos_tol'] = 0.001
                    config['goal_orien_tol'] = 0.001
                    self.arm_torso.configure_planner(config)
                    return 'succeeded'
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
            
        executionTime = (time.time() - self.startTime)
        with open('full_runs_timings.txt', 'a+') as f:
            f.write('Full run state machine execution time: ' + str(executionTime) + '\n')

        return 'failed'

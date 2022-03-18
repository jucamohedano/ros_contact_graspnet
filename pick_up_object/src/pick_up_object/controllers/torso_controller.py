from __future__ import print_function
import time

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import tf
from tf2_py import TransformException


class TorsoController:
    JOINT_MAX = 0.35

    def __init__(self):
        self._listener = tf.TransformListener()
        self._client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def sync_reach_to(self, joint1, tolerance=0.02):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["torso_lift_joint"]
        point = JointTrajectoryPoint()
        point.positions = [joint1]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)

        self._client.send_goal(goal)
        self._client.wait_for_result()
        # state = self._client.get_state()
        final_distance = self._wait_torso_stop()
        """
        sometimes it returns state 4 (aborted), idk the reason...
        always return true!
        """
        return abs(final_distance - joint1 - 0.59) < tolerance

    def _wait_torso_stop(self, need_times=2):
        rate = rospy.Rate(2)
        last_z = None
        z_no_change_times = 0
        while not rospy.is_shutdown():
            try:
                (trans, quaternion) = self._listener.lookupTransform('/torso_fixed_link', '/torso_lift_link',
                                                                     rospy.Time(0))
                z = trans[2]
                if last_z is None:
                    last_z = z
                else:
                    last_z = z
                    if abs(z - last_z) < 0.01:
                        z_no_change_times += 1
                        if z_no_change_times >= need_times:
                            return last_z
                    else:
                        rospy.logwarn("the torso is still moving, wait...")
                rate.sleep()
            except TransformException:
                rospy.sleep(0.1)
        return last_z


if __name__ == '__main__':
    rospy.init_node("torso_test", anonymous=True)
    _c = TorsoController()
    print(_c.sync_reach_to(0))
    print(_c.sync_reach_to(0.1))
    print(_c.sync_reach_to(0.35))

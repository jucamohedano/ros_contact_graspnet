import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class GripperController:
    JOINT_MAX = 0.04

    def __init__(self):
        self._client = actionlib.SimpleActionClient("gripper_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def sync_reach_to(self, joint1, joint2=None, time_from_start=1, wait=3):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        point = JointTrajectoryPoint()
        if joint2 is None:
            joint2 = joint1
        point.positions = [joint1, joint2]
        point.time_from_start = rospy.Duration(time_from_start)
        goal.trajectory.points.append(point)

        self._client.send_goal(goal)
        done = self._client.wait_for_result(rospy.Duration(wait))
        state = self._client.get_state()
        if done and state == actionlib.GoalStatus.SUCCEEDED:
            return True
        return state

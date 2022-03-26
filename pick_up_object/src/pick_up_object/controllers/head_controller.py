import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class HeadController:
    def __init__(self):
        self._client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def sync_reach_to(self, joint1, joint2, time_from_start=1, wait=10):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2]
        point.time_from_start = rospy.Duration(time_from_start)
        goal.trajectory.points.append(point)

        self._client.send_goal(goal)
        done = self._client.wait_for_result(rospy.Duration(wait))
        state = self._client.get_state()

        return done and state == actionlib.GoalStatus.SUCCEEDED


if __name__ == '__main__':
    rospy.init_node("head_test", anonymous=True)
    _head = HeadController()
    _head.sync_reach_to(0, -0.98)

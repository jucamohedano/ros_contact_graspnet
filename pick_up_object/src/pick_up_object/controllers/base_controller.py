import math
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf2_py import TransformException
from typing import Tuple, Optional, Union, Sequence


class BaseController:
    def __init__(self):
        self._listener = tf.TransformListener()
        self._goal_sent = False
        self._callback = None
        self._client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._velocity_publisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self._client.wait_for_server()

    def reg_callback(self, callback):
        self._callback = callback

    def _goto_done_cb(self, state, done):
        # Allow robot up to 60 seconds to complete task
        self._goal_sent = False
        if self._callback is not None:
            if done and state == GoalStatus.SUCCEEDED:
                self._callback(True)
            else:
                self._callback(state)

    def async_goto(self, position, radians=0, degrees=None):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        if degrees is not None:
            theta = math.radians(degrees)
        else:
            theta = radians
        quaternion = quaternion_from_euler(0, 0, theta)

        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': quaternion[2], 'r4': quaternion[3]}

        goal.target_pose.pose = Pose(Point(position[0], position[1], 0.000),
                                     Quaternion(quaternion['r1'], quaternion['r2'], quaternion['r3'], quaternion['r4']))

        rospy.loginfo("base is going to (%.2f, %.2f, %.2f) pose", position[0], position[1], theta)

        # Start moving
        self._goal_sent = True
        self._client.send_goal(goal, done_cb=self._goto_done_cb)

    def sync_reach_to(self, position, radians=0, degrees=None, wait=60):
        # type: (Optional[Sequence[float,float]], Optional[float], Optional[float], int) -> Union[bool,int]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        if degrees is not None:
            theta = math.radians(degrees)
        else:
            theta = radians
        quaternion = quaternion_from_euler(0, 0, theta)

        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': quaternion[2], 'r4': quaternion[3]}

        goal.target_pose.pose = Pose(Point(position[0], position[1], 0.000),
                                     Quaternion(quaternion['r1'], quaternion['r2'], quaternion['r3'], quaternion['r4']))

        rospy.loginfo("base is going to (%.2f, %.2f, %.2f) pose", position[0], position[1], theta)

        self._goal_sent = True
        self._client.send_goal(goal)

        done = self._client.wait_for_result(rospy.Duration(wait))
        self._goal_sent = False

        state = self._client.get_state()
        if done and state == GoalStatus.SUCCEEDED:
            return True
        return state

    def is_running(self):
        return self._client.get_state() == GoalStatus.PENDING or self._client.get_state() == GoalStatus.ACTIVE

    def get_current_pose(self):
        # type: () -> Tuple[float,float,float]
        while not rospy.is_shutdown():
            try:
                (trans, quaternion) = self._listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                x = round(trans[0], 2)
                y = round(trans[1], 2)
                euler = euler_from_quaternion(quaternion)
                yaw = round(euler[2], 2)
                return x, y, yaw
            except TransformException:
                rospy.sleep(0.1)

    def cancel(self):
        if self._goal_sent is True:
            state = self._client.get_state()
            if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
                self._client.cancel_goal()
            while True:
                if (self._client.get_state() in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                                                 GoalStatus.RECALLED, GoalStatus.SUCCEEDED]):
                    break
                time.sleep(0.5)
                self._goal_sent = False

    def linear_move(self, target_distance, linear_speed):
        desired_velocity = Twist()
        desired_velocity.linear.x = linear_speed
        desired_velocity.linear.y = 0
        desired_velocity.linear.z = 0
        desired_velocity.angular.x = 0
        desired_velocity.angular.y = 0
        desired_velocity.angular.z = 0

        rate = rospy.Rate(5)
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        while current_distance < target_distance:
            self._velocity_publisher.publish(desired_velocity)
            rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_distance = abs(linear_speed) * (t1 - t0)

        desired_velocity.linear.x = 0
        self._velocity_publisher.publish(desired_velocity)


if __name__ == '__main__':
    rospy.init_node("base_test", anonymous=True)
    _base = BaseController()
    _base.sync_reach_to((1.73, -0.03), -1.57)

# 6 DoF grasp synthesis project

Three ROS packages have been developed to implement two different solutions to grasping:

  1. [contact_graspnet](https://github.com/jucamohedano/ros_contact_graspnet/tree/main/contact_graspnet) -- ROS Wrapper of [Contact-Graspnet](https://github.com/NVlabs/contact_graspnet) with modifications made to switch from a Panda gripper to the TIAGo gripper

  2. [geometric_grasp](https://github.com/jucamohedano/ros_contact_graspnet/tree/main/geometric_grasp) - A ROS package with an analytical approach to grasping.

Both of these packages are used in the *pick_up_object* state machine ros package.

## Prerequisites

Kinetic docker container

Melodic docker container with TensorFlow and Nvidia support


## How to run it

1\. `roslaunch contact_graspnet contact_graspnet.launch` or `roslaunch geometric_grasp geometric_grasp.launch`

2\. `roslaunch pick_up_object pick_up_object.launch`

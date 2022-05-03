# 6 DoF grasp synthesis project

Three ROS packages have been developed to implement two different solutions to grasping:

1\.  Contact-Graspnet (https://github.com/NVlabs/contact_graspnet) -- *contact_graspnet* ros package

2\.  Analytical grasping approach -- *gemetric_grasp* ros package

Both of these packages are used in the *pick_up_object* state machine ros package.

## Prerequisites

Kinetic docker container

Melodic docker container with TensorFlow and Nvidia support


## How to run it

1\. `roslaunch contact_graspnet contact_graspnet.launch` or `roslaunch geometric_grasp geometric_grasp.launch`

2\. `roslaunch pick_up_object pick_up_object.launch`

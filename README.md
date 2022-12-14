# 6 DoF grasp synthesis project

Three ROS packages have been developed to implement two different solutions to grasping:

  1. [contact_graspnet](https://github.com/jucamohedano/ros_contact_graspnet/tree/main/contact_graspnet) - ROS Wrapper of [Contact-Graspnet](https://github.com/NVlabs/contact_graspnet) with modifications made to switch from a Panda gripper to the TIAGo gripper. This method uses a learning based approach.

  2. [geometric_grasp](https://github.com/jucamohedano/ros_contact_graspnet/tree/main/geometric_grasp) - A ROS package with an analytical approach to grasping.

  3. [pick_up_object](https://github.com/jucamohedano/ros_contact_graspnet/tree/main/pick_up_object) - State machine that allows for the execution of both grasping methods.

## Prerequisites

Kinetic docker container.

Melodic docker container with TensorFlow and Nvidia support. Refer to [tiago_docker_tensorflow](https://github.com/jucamohedano/tiago_docker_tensorflow).


## How to run it

  1\. Choose what launch file to launch depending on what method you would like to test: 
      
      `roslaunch contact_graspnet contact_graspnet.launch`
      
      `roslaunch geometric_grasp geometric_grasp.launch`

  2\. `roslaunch pick_up_object pick_up_object.launch`
  
  3\. In order to switch the different methods of grasping, it must be addressed in the [**generate_grasps**](https://github.com/jucamohedano/ros_contact_graspnet/blob/main/pick_up_object/src/pick_up_object/states/generate_grasps.py) state.

## Pick up pipeline

Overview of the full pipeline to grasp an object.

![System overview](https://raw.githubusercontent.com/jucamohedano/ros_contact_graspnet/master/docs/system_overview.png)

## Image demonstration of Contact-GraspNet inference

![Contact-GraspNet predictions](https://raw.githubusercontent.com/jucamohedano/ros_contact_graspnet/master/docs/contact_graspnet_inference_example.png)

## Image demonstration of the Analytical approach grasps prediction

![Geometric-grasping predictions](https://raw.githubusercontent.com/jucamohedano/ros_contact_graspnet/master/docs/analytical_method_example.png)

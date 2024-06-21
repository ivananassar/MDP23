# MDP23
Automated drone for detecting, tracking and predefining a trajectory towards solar panels using PX4-Autopilot, ROS1 Noetic, Gazebo 11, Machine Learning algorithms.


The AUTO folder contains the concept of tracking a solar panel using the Intel RealSense D435i camera, inspired by another public GitHub repository. Although the code is unfinished, it outlines the main idea and purpose. The primary files that need adjustments are run_yolo, camera, and track. The original code is based on YOLO, but our project uses UNet for segmentation. Therefore, adapting the tracking system to use run_unet instead of run_yolo is challenging. We have started making these changes but have not yet tested them. The segmentation file contains the machine learning model trained to detect solar panels and convert their masks into bounding boxes. It should be carefully integrated into the run_yolo file.

Additionally, the trajectory_control folder is based on INP Grenoble's research and was used for testing purposes to verify that the installation between ROS 1, PX4, and Gazebo 11 was working properly.

Finally, the technical logistics needed are:

Ubuntu 22.04: Follow the PX4 main guide here: https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html.
Mavlink, mavros libraries inside ROS 1 workspace directories:[ Mavlink](https://mavlink.io/en/getting_started/use_libraries.html), [Mavros](https://github.com/mavlink/mavros).
ROS1 installation: Follow the guide here : https://docs.px4.io/main/en/ros/ros1.html.
Rviz: Installation for visualization testing purposes.
Python: and the ML libraries.
PX4 Firmware (without Gazebo built-in): Follow the guide here: https://docs.px4.io/main/en/.
Gazebo 11: Preferably stand-alone, with knowledge of its plugin for the Intel RealSense D435i camera and iris drone. More information here:https://classic.gazebosim.org/ , https://github.com/pal-robotics/realsense_gazebo_plugin
Catkin: Basic knowledge of how Catkin works.
QGround Control: Not used in our project but can be used if necessary for more technical aspects.
The GitHub link we started from: https://github.com/HKPolyU-UAV/AUTO.

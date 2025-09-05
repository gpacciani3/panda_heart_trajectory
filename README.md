# Executing a ”Heart-Shaped Trajectory in the XZ-plane” with the Franka Robot in Gazebo

This repository shows how to execute a heart-trajectory using MoveIt and how to simulate it with the Franka robot in the Gazebo simulation environment.

## Commands for the execution:
```
- cd ~/catkin_ws/src
'Copy and paste in catkin_ws/src the folder src, the file CMakeList.txt and the package.xml.'
- cd ~/catkin_ws
- catkin_make
- source devel/setup.bash
- roslaunch panda_moveit_config demo_gazebo.launch
'In a new terminal run:'
- rosrun panda_heart_trajectory heart_trajectory
```

## Commands for data collection:
```
'In terminal 1:'
- roslaunch panda_moveit_config demo_gazebo.launch
'In terminal 2:'
- rosbag record \
  /franka_state_controller/joint_states \
  /franka_state_controller/franka_states \ -O ~/Industrial_robotics_project/Rosbag_data_collection/data_collected_franka.bag
  'substitute "Industrial_robotics_project/Rosbag_data_collection" with the directory chosen to visualize the bag file'
'In terminal 3:'
- rosrun panda_heart_trajectory heart_trajectory
'Once the terminal 3 is compiled, stop recording in terminal 2 with Ctrl+C. Finally, stop the terminal 1.'
```
## Instructions for the plots
Download the "MATLAB" folder and open the two files "Data_collected_joints.m" and "Data_collected_EE_trajectories.m". Run these files in which the data are extracted and visualised in relative plots.


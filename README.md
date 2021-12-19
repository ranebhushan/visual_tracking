# Object Tracking using Arm Mounted Camera

Course Project for RBE501 - Robot Dynamics (Fall 2021)

Master of Science in Robotics Engineering at Worcester Polytechnic Institute

## Team Members
- Bhushan Ashok Rane
- Gaurang Manoj Salvi
- Rutwik Rajesh Bonde
- Yash Rajendra Patil

### Simulation Video

https://user-images.githubusercontent.com/34753789/146660347-52cc2bac-a6a6-4151-ab33-855ae760a66f.mov

## Project Description

### Dependencies
- ROS Distro : Noetic Ninjemys v1.15.13
- Operating System : Ubuntu 20.04

### Usage Guidelines

To run this project, initially you need to clone this repository in your local computer.

```
https://github.com/ranebhushan/rbe501_project.git
```

Copy the contents of the `/ros` folder of this repository to `/src` of your ROS workspace.

Build your ROS workspace using either `catkin_make` or `catkin build` (depending on how your workspace was created) 

Open your ROS workspace in terminal and run the following command:

```
source devel/setup.bash
``` 

Launch the simulation environment using the following command:

```
roslaunch obj_tracking spawn_robot_and_obj.launch
```
> Note: Sometimes, the simulation environment remains paused for some initial number of seconds and once it's normal, the robot moves to it's home position.

Once the simulation environemtn is active, launch the object tracking control node using the following command:

```
rosrun obj_tracking node_obj_tracking.py
```

Now, the robot will start tracking the object.

You can move the object by displacing it within the frame of the camera, and then the robot will start following the object.
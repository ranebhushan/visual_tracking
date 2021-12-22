# Object Tracking using Arm Mounted Camera

Course Project for RBE501 - Robot Dynamics (Fall 2021)

Master of Science in Robotics Engineering at Worcester Polytechnic Institute

### Team Members
- Bhushan Ashok Rane
- Gaurang Manoj Salvi
- Rutwik Rajesh Bonde
- Yash Rajendra Patil

## Simulation

https://user-images.githubusercontent.com/34753789/146660906-a69beaf6-68b3-4423-bd20-78fc39d483c7.mov

## Project Description

### Dependencies
- ROS Distro : Noetic v1.15.13
- OS : Ubuntu 20.04 LTS

### Usage Guidelines

- To run this project, you will need to clone this repository in your local project folder using the following command:

    ```
    git clone https://github.com/ranebhushan/visual_tracking.git
    ```

- Copy the contents of `/ros` folder of this repository to `/src` of your ROS workspace.

- Build your ROS workspace using either `catkin_make` or `catkin build` (depending on how your workspace was created) 

- Open your ROS workspace in terminal and run the following command:

    ```
    source devel/setup.bash
    ``` 

- Launch the Gazebo simulation environment using the following command:

    ```
    roslaunch obj_tracking spawn_robot_and_obj.launch
    ```
    > Note: Sometimes, the simulation environment remains paused for some initial number of seconds and once it's normal, the robot moves to it's home position.

- Once the simulation environment is active, launch the object tracking control node using the following command:

    ```
    rosrun obj_tracking node_obj_tracking.py
    ```
    Now, the robot will start tracking the object.

- You can move the object by displacing it within the frame of the camera, and then robot will start following the object.

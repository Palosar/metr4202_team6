# METR4202_TEAM_6 - Robotics Project

## Preface
This code is made by Team 6 in the course METR4202 Robotics & Automation at University of Queensland. 

## Introduction
This project succeeded in controlling a 4R Robot to pick up cubes and sort them by colour from a conveyor belt. It is run in python using the software architecture Robotic Operative System (ROS). A state machine is implemented with the following states:

- **Prediciton**: This is the default arm configuration where there is full visibility of the conveyer belt. The camera detects the aruco tags on the block and identifies when the cubes are stationary. 
- **Pickup**: When the cubes are stationary the pickup state is initialized. The aruco tags give the position of the cube which is sent to the inverse kinematics code, supplying joint angles for the robot. Trajectory movement control is implemented to avoid collisions. The inverse kinematic solution where the gripper is at its most vertical position is chosen. Additionally, the robot will stop at intermediate poses above the cubes before grabbing and after picking up cube to avoid pushing away other cubes. 
- **Colour_check**: After the cube has been picked up it will be lifted up towards the camera at an angle, showing the side of the cube. 
This allows for the colour detection implementation to detect the color of the cube. As the robot can not lift up the cube to fully cover the camera, a sector of the image is used in colour detection. 
- **Drop_off**: After detecting the colour in the previous state, the robot will place the cube in the right drop off zone according to its colour.

## Usage
In order to run the code the following has to be launched in 5 different terminals. 
```
#Terminal 1: (needs a master roscore)
	roscore

#Terminal 2: (main camera, need to be in catkin_ws folder)
	source devel/setup.bash
	echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
	rosrun ximea_ros ximea_demo
	
#Terminal 3: (aruco tag detection). Use the serial number of the camera.
	source devel/setup.bash
	roslaunch ximea_ros ximea_aruco.launch serial:=31703851 
	
#Terminal 4: (main project)
	source devel/setup.bash
	sudo pigpiod
	roslaunch robot_arm_controller robot_arm.launch
	
Terminal 5: (colour)
	source devel/setup.bash
	rosrun ximea_color example_camera.py

```
## Acknowledgements
We would like to acknowledge the METR4202 tutoring team for supplying the skeleton code for our project. Additionally, they provided code for colour detection. (anything more?)
## Sources

Functions:
- invk() - theory based on (Siciliano, B et al, 2009) “Robotics Modelling, Planning and Control”. 

- RpToTrans() - Modern Robotics library

- euler_from_quaternion(x, y, z, w) - Quaternion to Euler transform:
https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/

- colour_check() - METR4202's Github. https://github.com/UQ-METR4202

# Factory Calibration (WIP)

## Overview
This package contains two ROS workspace one is for depthai capture and calibration node and another is for Interbotix ViperX 300 Robot Arm 6DOF ([KIT-VIPX300-6DOF](https://www.trossenrobotics.com/viperx-300-robot-arm-6dof.aspx)) arm bot control using moveit.

The reason for two workspace is because we need python3 for depthai and python2 for interbotix moveit interface provided by interbotix [here](https://github.com/Interbotix/interbotix_ros_arms/tree/melodic). 

![DepthAI Automated Calibration Example](https://user-images.githubusercontent.com/32992551/98047318-f251d700-1de8-11eb-8c56-ca31c038a1aa.jpg)
![DepthAI Automated Calibration Example](https://user-images.githubusercontent.com/32992551/98047330-f4b43100-1de8-11eb-8315-3cad43df8966.jpg)


## Python3 on ROS Melodic
Follow thw video [here](https://youtu.be/oxK4ykVh1EE) till 30 min. and then pip install depthai.  

## Example Calibration Result

![Successful Automated Calibration](https://user-images.githubusercontent.com/32992551/98423514-0ba68d80-204c-11eb-8562-119cefe3c158.jpg)
![Calibrated Object Localization](https://user-images.githubusercontent.com/32992551/98423642-87083f00-204c-11eb-8445-6da38587797c.jpg)


# Factory Calibration (WIP)

## Overview
This package contains two ROS workspace one is for depthai capture and calibration node and another is for Interbotix ViperX 300 Robot Arm 6DOF ([KIT-VIPX300-6DOF](https://www.trossenrobotics.com/viperx-300-robot-arm-6dof.aspx)) arm bot control using moveit.

The reason for two workspace is because we need python3 for depthai and python2 for interbotix moveit interface provided by interbotix [here](https://github.com/Interbotix/interbotix_ros_arms/tree/melodic). 

![DepthAI Automated Calibration Example](https://user-images.githubusercontent.com/32992551/98047318-f251d700-1de8-11eb-8c56-ca31c038a1aa.jpg)
![DepthAI Automated Calibration Example](https://user-images.githubusercontent.com/32992551/98047330-f4b43100-1de8-11eb-8315-3cad43df8966.jpg)


## Python3 on ROS Melodic
Follow the video [here](https://youtu.be/oxK4ykVh1EE) till 30 min. and then pip install depthai.  

## Workspace setup procedure on Ubuntu 18
1. Install ROS melodic from [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. install dependencies &#8594; `sudo apt install git cmake python-pip python3-pip`
3. install python virual env using `sudo pip install virtualenv`
4. go to home directory `cd ~`
5. clone the repo 
`
git clone https://github.com/luxonis/Factory-calibration-DepthAI.git
`
### Calibration Node setup
1. go to python_ws directory `cd ~/Factory-calibration-DepthAI/python3_ws`
2. create a virtual environment using `virtualenv py3venv --python=python3`
3. Activate the virutalenv using `source py3venv/bin/activate`
4. go to repository main directory  `cd ~/Factory-calibration-DepthAI`
5. install the libraries in virtual env using `python -m pip install -r requirements.txt`
6. go back to python3_ws workspace `cd ~/Factory-calibration-DepthAI/python3_ws`
7. build the calibration package using `catkin_make`
8. add the ros package sourcing to .bashrc `echo "source ~/Factory-calibration-DepthAI/python3_ws/devel/setup.bash" >> ~/.bashrc`
9. USB rules for Movidius `echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules`
10. reload the usb rules `sudo udevadm control --reload-rules && udevadm trigger`

### Interbotix viperx 300s arm Node setup
1. Open a new terminal
2. install rosdep `sudo apt install python-rosdep2`
3. open interbotix_ws workspace `cd ~/Factory-calibration-DepthAI/interbotix_ws`
4. initialize rosdep using `sudo rosdep init` 
5. build the project using `catkin_make` (will throw errors thats fine)
6. `rosdep update` updates the required dependencies 
7. install the missing dependencies using `rosdep install --from-paths src --ignore-src -r -y`
8. Build the package again. `catkin_make`
9. Installing python dependency not declared in package.xml `python -m pip install rospkg pygame`
10. add pacakgae paths to `echo "source ~/Factory-calibration-DepthAI/interbotix_ws/devel/setup.bash" >> ~/.bashrc`
11. load usb rules `sudo cp src/interbotix_sdk/10-interbotix-udev.rules /etc/udev/rules.d`
12. Reload the usb rules `sudo udevadm control --reload-rules && udevadm trigger`
 

## Running calibration
1. Open two terminals say #1 and #2
2. in #1 start virtual env using `source ~/Factory-calibration-DepthAI/python3_ws/py3venv/bin/activate`
3. In #1 start calibration node using `roslaunch calibration depthai_calib.launch enable_IMU_test:=false` (set enable_IMU_test:=true if you want to test connection to IMU)
4. In #2 use the following command to launch the arm `roslaunch interbotix_moveit_interface moveit_interface.launch robot_name:=vx300s use_actual:=true  dof:=6`
5. Alternative to step 4. An Applications icon called `arm node` has been created during build which can also be used to launch the arm.



## Example Calibration Result

![Successful Automated Calibration](https://user-images.githubusercontent.com/32992551/98423514-0ba68d80-204c-11eb-8562-119cefe3c158.jpg)
![Calibrated Object Localization](https://user-images.githubusercontent.com/32992551/98423642-87083f00-204c-11eb-8445-6da38587797c.jpg)


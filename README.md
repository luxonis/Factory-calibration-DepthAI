# DepthAI Factory Calibration

## Overview
This package contains two ROS workspace one is for depthai capture and calibration node and another is for Interbotix ViperX 300 Robot Arm 6DOF ([KIT-VIPX300-6DOF](https://www.trossenrobotics.com/viperx-300-robot-arm-6dof.aspx)) arm bot control using moveit.

This repository contains the charuco board used in this calibration setup.

The reason for two workspace is because we need python3 for depthai and python2 for interbotix moveit interface provided by interbotix [here](https://github.com/Interbotix/interbotix_ros_arms/tree/melodic). 

![DepthAI Automated Calibration Example](https://user-images.githubusercontent.com/32992551/103242234-c0ee1700-4912-11eb-881e-93b6a6843afe.jpg)

![DepthAI Automated Calibration Example](https://user-images.githubusercontent.com/32992551/103242260-e0853f80-4912-11eb-96b6-887ef9f0f662.png)
![DepthAI Automated Calibration Example](https://user-images.githubusercontent.com/32992551/103242359-27733500-4913-11eb-9f19-a818bc7fc5e6.jpg)


## Python3 on ROS Melodic
Follow the video [here](https://youtu.be/oxK4ykVh1EE) till 30 min. and then pip install depthai.  

## Workspace setup procedure on Ubuntu 18.04
1. Install ROS melodic from [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. install dependencies &#8594; `sudo apt install git cmake python-pip python3-pip`
3. install python virual env using `python3 -m pip install virtualenv`
4. Go to home directory `cd ~`
5. Clone the repo ` git clone https://github.com/luxonis/Factory-calibration-DepthAI.git`

### Calibration Node setup
For convenience, you can use the `setup_calibration_node.py` script, which executes all the commands below automatically
```
$ python3 setup_calibration_node.py
```
1. Go to python_ws directory `cd ~/Factory-calibration-DepthAI/python3_ws`
2. Create a virtual environment using `virtualenv py3venv --python=python3`
3. Activate the virutalenv using `source py3venv/bin/activate`
4. Go to the repository main directory  `cd ~/Factory-calibration-DepthAI`
5. Install the libraries in virtual env using `python -m pip install -r requirements.txt`
6. Go back to python3_ws workspace `cd ~/Factory-calibration-DepthAI/python3_ws`
7. Build the calibration package using `catkin_make`
8. Add the ros package sourcing to .bashrc `echo "source ~/Factory-calibration-DepthAI/python3_ws/devel/setup.bash" >> ~/.bashrc`
9. USB rules for Movidius `echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules`
10. Reload the usb rules `sudo udevadm control --reload-rules && udevadm trigger`

### Interbotix viperx 300s arm Node setup
For convenience, you can use the `setup_calibration_node.py` script, which executes all the commands below automatically

```
$ python3 setup_calibration_node.py
```

1. Open a new terminal
2. Install rosdep `sudo apt install python-rosdep2`
3. Open interbotix_ws workspace `cd ~/Factory-calibration-DepthAI/interbotix_ws`
4. Initialize rosdep using `sudo rosdep init` 
5. Build the project using `catkin_make` (will throw errors thats fine)
6. `rosdep update` updates the required dependencies 
7. Install the missing dependencies using `rosdep install --from-paths src --ignore-src -r -y`
8. Build the package again. `catkin_make`
9. Installing python dependency not declared in package.xml `python -m pip install rospkg pygame`
10. Add pacakage paths to `echo "source ~/Factory-calibration-DepthAI/interbotix_ws/devel/setup.bash" >> ~/.bashrc`
11. Load usb rules `sudo cp src/interbotix_sdk/10-interbotix-udev.rules /etc/udev/rules.d`
12. Reload the usb rules `sudo udevadm control --reload-rules && udevadm trigger`
 

## Running calibration using Interbotix ViperX 300 Robot Arm
1. Open two terminals say #1 and #2
2. In #1 start virtual env using `source ~/Factory-calibration-DepthAI/python3_ws/py3venv/bin/activate`
3. In #1 start calibration node using `roslaunch calibration depthai_calib.launch brd:=bw1098obc` (change the board name and add the config file if it is not available)
4. In #2 use the following command to launch the arm `roslaunch interbotix_moveit_interface moveit_interface.launch robot_name:=vx300s use_actual:=true  dof:=6`
5. Alternative to step 4. An Applications icon called `arm node` has been created during build which can also be used to launch the arm. use `catkin_make install` in `interbotix_ws` to enable this.

## Running calibration without arm Robot(Not Recommended. Please use the python script [here](https://docs.luxonis.com/en/latest/pages/calibration/)).
1. Open three terminals say #1, #2 and #3
2. In #1 start virtual env using `source ~/Factory-calibration-DepthAI/python3_ws/py3venv/bin/activate`
3. In #1 start calibration node using `roslaunch calibration depthai_calib.launch brd:=bw1098obc`
4. Select the CCM in the window. (This is a local logging selection window and doesn't affect the calibration in any way as of now but needed to proceed.)
5. Run all the following commands in #3 terminal
6. Run `rosservice call /device_status "name: ''"` first to check connection and make window available.
7. In #2 (if using rgb_calibration branc) start rqt visualization `rqt_image_view` (at the top right corner of the window select `color`)
8. Run `rosservice call /capture_checkerboard "name: 'right_1'"` execute this command to capture image when in position. (need to run this for different positon and while doing so change the name in the command.) And while doing so check the window created by #2 terminal to see if the image is sharp and not out of focus.
9. After capturing 6+ sharp images in different position execute the following command to calibrate and write to eeprom. `rosservice call /calibrate_stereo "name: 'charuco'"`
10. Alternative to step 4. An Applications icon called `arm node` has been created during build which can also be used to launch the arm.

## Example Pass and Fail Results in the Calibration GUI

![Image from iOS](https://user-images.githubusercontent.com/32992551/111221587-5018db80-85a0-11eb-8991-43cc7b35a317.jpg)
![Image from iOS (1)](https://user-images.githubusercontent.com/32992551/111221597-51e29f00-85a0-11eb-91fa-eeed4f26a90d.jpg)

## Example Calibration Result

![Successful Automated Calibration](https://user-images.githubusercontent.com/32992551/98423514-0ba68d80-204c-11eb-8562-119cefe3c158.jpg)
![Calibrated Object Localization](https://user-images.githubusercontent.com/32992551/98423642-87083f00-204c-11eb-8445-6da38587797c.jpg)

## Hardware Components and Costs

 - Interbotix ViperX 300 Robot Arm 6DOF ([KIT-VIPX300-6DOF](https://www.trossenrobotics.com/viperx-300-robot-arm-6dof.aspx)): $5,000
 - Intel NUC 8 ([NUC8i5BEH](https://www.amazon.com/gp/product/B07GX59NY8/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)): $300
 - Corsair Memory Kit 16GB ([(2x8GB) DDR4 2400MHz SODIMM Memory](https://www.amazon.com/gp/product/B019MRBKYG/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)): $75
 - Samsung 970 EVO Plus SSD 500GB ([MZ-V7S500B/AM](https://www.amazon.com/gp/product/B07M7Q21N7/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)): $90
 - 4-Port USB 3.0 Hub ([HB-UM43](https://www.amazon.com/dp/B00JX1ZS5O/ref=cm_sw_r_oth_api_glc_fabc_gi42FbR87JEAW?_encoding=UTF8&psc=1)): $11
  (The USB Hub allows shorter cable to be plugged in on-arm.)
 
 **TOTAL**: $5,476 

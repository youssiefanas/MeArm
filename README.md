# MeArm pocket Size Manipulator

MeArm is a 3-DOF robot arm, that is controlled by arduino and connected to matlab using ROS. 
In this project both forward and inverse kinematics were constructed to model the robot motion towards the desired postions.

## Build
 
test

Upload the ino code on the arduino board
download the Matlab Excutable GUI file
You can run this roslaunch file that will excute all the ROS nodes, and use the gui to control the robot

# ROS 

ROS Neotics
rosserial for arduino

# Prerequisits installations

## Using pip

### Peter CorKe Robotics toolbox for Python

pip3 install roboticstoolbox-python

### Spatial Maths for Python

pip install spatialmath-python

### install tkinter

pip install tk

# ROS pkg installation_
 
 git clone git@github.com:youssiefanas/MeArm.git

Copy the meArm_py folder to your ROS workspace (ex: catkin_ws)

then make go to your ws directory and run "catkin_make" in you terminal

Run : roscore
    rosrun meArm_py mearm_gui.py 
    rosrun rosserial_arduino serial_node.py /dev/ttyACM0

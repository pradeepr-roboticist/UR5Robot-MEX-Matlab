# UR5Display
![alt text](https://raw.githubusercontent.com/pradeepunique1989/UR5Display/master/docs/image.png)

This repository contains a MATLAB class to display an UR5 robot.
There are many ways to visualize a robot model in MATLAB.
This class does it efficiently by utilizing "hgtransform" objects provided by MATLAB.

I hope this class is of use to anyone wanting to:
1) Visualize the pose of an UR5 robot.
2) Visualize and manipulate STL objects efficiently from within MATLAB.

I am grateful to the authors of the following packages:

1) Example MATLAB class wrapper for a C++ class by Oliver Woodford
https://www.mathworks.com/matlabcentral/fileexchange/38964-example-matlab-class-wrapper-for-a-c-class

2) stlTools by Pau Micó
https://www.mathworks.com/matlabcentral/fileexchange/51200-stltools?focused=3878420&tab=function

3) universal_robot package by Kelsey Hawkins (kphawkins@gatech.edu)
https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_kinematics


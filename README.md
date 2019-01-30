# UR5 Kinematics and Display
![image of figure window](https://raw.githubusercontent.com/pradeepunique1989/UR5Display/master/docs/image.png)

This repository contains two MATLAB classes.
1) UR5Display : A class to visualize a robot model using the corresponding STL files.
2) UR5Kinematics : A wrapper class to compute forward and inverse kinematics solutions via a MEX interface to C++ code.

There are many ways to visualize a robot model in MATLAB.
This class does it efficiently by utilizing graphics objects provided by MATLAB.
The key advantage is that the robot model can be efficiently articulated by either the callback function or a call to draw_configuration.

I hope this class is of use to anyone wanting to:
1) Visualize the pose of an UR5 robot.
2) Visualize and manipulate STL objects efficiently from within MATLAB.

## Credits
I am grateful to the authors of the following packages:

1) Example MATLAB class wrapper for a C++ class by Oliver Woodford
https://www.mathworks.com/matlabcentral/fileexchange/38964-example-matlab-class-wrapper-for-a-c-class

2) stlTools by Pau Micó
https://www.mathworks.com/matlabcentral/fileexchange/51200-stltools?focused=3878420&tab=function

3) universal_robot package by Kelsey Hawkins (kphawkins@gatech.edu)
https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_kinematics

## Usage
1) Download or clone this repo into your folder.
2) Run the script "run_me.m".
3) Star this repo if you like it.

## Some tips
1) You need to have set a MEX compiler for this to work
2) This was tested in MATLAB 2018b. But, it should work for any recent version of MATLAB.

## Author

**Pradeep Rajendran**

* [github/pradeepunique1989](https://github.com/pradeepunique1989)

## License

Copyright © 2019 [Pradeep Rajendran](https://github.com/pradeepunique1989)
Released under the [GNU General Public License](https://github.com/pradeepunique1989/UR5Robot/blob/master/LICENSE).

% File              : compile_ur5kinematics.m
% Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
% Date              : 06.10.2018
% Last Modified Date: 06.10.2018
% Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>

% We have to pass the compiler directive "UR5_PARAMS" so that it enables the appropriate definitions in ur_kin.cpp.
mex -R2018a -DUR5_PARAMS -I./ -I./class_interface/class_interface/ ur5_kinematics_mex.cpp ur_kin.cpp

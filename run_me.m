% File              : run_me.m
% Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
% Date              : 06.10.2018
% Last Modified Date: 29.01.2019
% Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>

addpath(genpath(get_this_dir()));
%% COMPILE MEX CODE
cd ur_kinematics;
run compile_ur5kinematics.m
cd ..

%% BOILERPLATE
f = figure();
axis equal;
grid on;
grid minor;
camlight;
material metal;
axis([-0.6 0.6 -0.6 0.6 0 1]*1);

ur5_disp = UR5Display(f);

%% MOVING TO A GIVEN CONFIGURATION
ur5_disp.draw_configuration([-0.95 -0.85 -1.2 0.95 0.95 0.6]);

%% KEYBOARD CONTROL

fprintf("\n*************** KEYBOARD CONTROL **********************\n");
fprintf("Press the following keys for joint motion\n");
fprintf("q,w,e,r,t,y for incrementing joint angles 1 through 6\n");
fprintf("a,s,d,f,g,h for decrementing joint angles 1 through 6\n");
fprintf("u for increasing the granularity of joint angle change\n");
fprintf("j for decreasing the granularity of joint angle change\n");
fprintf("\n***********************************************\n");
 
%% FORWARD AND INVERSE KINEMATICS

% Forward kinematics
ur5_kin = UR5Kinematics();
result = ur5_kin.forward_kinematics([0 0 0 0 0 0]); % obtains the transform frames of all links
result.transform_matrices

% Inverse kinematics
T_ee = eye(4);
T_ee(1:3, 1:3) = eye(3); % this sets the rotation component of the end-effector
T_ee(1:3, 4) = [0.6; 0.1; 0.6]; % this sets the position component of the end-effector
[ik_sols] = ur5_kin.inverse_kinematics(T_ee, 0); % computes the IK solutions to reach end-effector configuration T_ee

ur5_disp.draw_configuration(ik_sols(2, :)); % visualize one of the solutions

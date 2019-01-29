% File              : UR5Kinematics.m
% Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
% Date              : 06.10.2018
% Last Modified Date: 13.12.2018
% Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>
classdef UR5Kinematics < handle
    properties (Constant)
        REQUEST_FORWARD_KINEMATICS_CMD_ID = uint64(0);
        REQUEST_INVERSE_KINEMATICS_CMD_ID = uint64(1);
        REQUEST_COLLISION_BALLS_CMD_ID = uint64(2);
    end
    
    methods (Static)
        %% Request forward kinematics
        function varargout = forward_kinematics(varargin)
            if isempty(varargin{1})
                error('Joint angles required.');
            end
            [varargout{1:nargout}] = ur5_kinematics_mex(UR5Kinematics.REQUEST_FORWARD_KINEMATICS_CMD_ID, varargin{:});
            
        end
        %% Request inverse kinematics
        function [ik_sols] = inverse_kinematics(T_ee, ~)
            if nargin < 2
                error('Transform of end-effector and end-effector angle required');
            end
            q6_des = 0;
            [ik_sols_tmp, num_valid_sols] = ur5_kinematics_mex(UR5Kinematics.REQUEST_INVERSE_KINEMATICS_CMD_ID, T_ee', q6_des);
            ik_sols_valid = ik_sols_tmp(:, 1:num_valid_sols)';
            ik_sols = zeros(size(ik_sols_valid));
            ik_sol_count = 0;
            for mk = 1 : size(ik_sols_valid, 1)
                q_tent = wrapToPi(ik_sols_valid(mk, :));
                solution_duplicate = false;
                for rk = 1 : size(ik_sols, 1)
                    q_test = ik_sols(rk, :);
                    dist_q = norm(q_test-q_tent);
                    if dist_q < 1e-4
                        solution_duplicate = true;
                        break;
                    end
                end
                if false == solution_duplicate
                    ik_sol_count = ik_sol_count + 1;
                    ik_sols(ik_sol_count, :) = q_tent;
                end
            end
            ik_sols(ik_sol_count+1:end, :) = [];
        end
        %% Request forward kinematics
        function varargout = collision_balls(varargin)
            if isempty(varargin{1})
                error('Joint angles required.');
            end
            [varargout{1:nargout}] = ur5_kinematics_mex(UR5Kinematics.REQUEST_COLLISION_BALLS_CMD_ID, varargin{:});
        end
        
        %         %% Request dummy function
        %         function varargout = dummy(varargin)
        %             [varargout{1:nargout}] = ur5_kinematics_mex(UR5Kinematics.DUMMY_ID, varargin{:});
        %         end
    end
end

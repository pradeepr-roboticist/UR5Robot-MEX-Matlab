% File              : UR5Display.m
% Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
% Date              : 06.10.2018
% Last Modified Date: 07.10.2018
% Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>
classdef UR5Display < handle
    properties (Constant)
        JACOBIAN_MODE = 0;
        JOINT_MODE = 1;
    end
    properties
        body_color
        q_display
        ur5_kin
    end
    properties (Access = private)
        ws_figure
        key_listener
        delete_listener

        q_delta
        
        v_base
        f_base
        n_base
        
        v_shoulder
        f_shoulder
        n_shoulder
        
        v_upperarm
        f_upperarm
        n_upperarm
        
        v_forearm
        f_forearm
        n_forearm
        
        v_wrist1
        f_wrist1
        n_wrist1
        
        v_wrist2
        f_wrist2
        n_wrist2
        
        v_wrist3
        f_wrist3
        n_wrist3
        
        v_tool
        f_tool
        n_tool      
        
        offset_1
        offset_2
        offset_3
        offset_5
        offset_6
        offset_7
        
        offset_tool
        
        ur5_base
        ur5_group
        
        g_base
        h_base
        
        g_shoulder
        h_shoulder
        
        g_upperarm
        h_upperarm

        g_forearm 
        h_forearm 

        g_wrist1 
        h_wrist1 

        g_wrist2 
        h_wrist2 

        g_wrist3 
        h_wrist3
        
        g_tool
        h_tool
        
        h_closest_point
        
        h_all_links
        

        control_mode
        
        precomp_matrices
        
    end
    methods (Access = private)
        function key_press_cb(obj, ~, event_data)
            
            if 'm' == event_data.Key
                obj.control_mode = UR5Display.JACOBIAN_MODE;
                fprintf('\nJACOBIAN_MODE\n');
                return;
            elseif 'n' == event_data.Key
                obj.control_mode = UR5Display.JOINT_MODE;
                fprintf('\nJOINT_MODE\n');
                return;
            end
            qd = zeros(1, 6);
            delta = obj.q_delta;

            switch event_data.Key
                case 'q'
                    qd(1) = delta;
                case 'a'
                    qd(1) = -delta;
                case 'w'
                    qd(2) = delta;
                case 's'
                    qd(2) = -delta;
                case 'e'
                    qd(3) = delta;
                case 'd'
                    qd(3) = -delta;
                case 'r'
                    qd(4) = delta;
                case 'f'
                    qd(4) = -delta;
                case 't'
                    qd(5) = delta;
                case 'g'
                    qd(5) = -delta;
                case 'y'
                    qd(6) = delta;
                case 'h'
                    qd(6) = -delta;
                case 'u'
                    obj.q_delta = obj.q_delta * 1.1;
                case 'j'
                    obj.q_delta = obj.q_delta / 1.1;
            end
            
            if UR5Display.JOINT_MODE == obj.control_mode
                q_diff = qd;
            elseif UR5Display.JACOBIAN_MODE == obj.control_mode
                % TODO
            end
            
            obj.q_display = obj.q_display + q_diff;

            obj.draw_configuration(obj.q_display);
            clipboard('copy', obj.q_display);

        end

        function init_display(obj)
            name = '';
            obj.h_base = stlPlot(obj.v_base, obj.f_base, name, 'FaceColor', obj.body_color, 'Parent', obj.g_base);
            obj.h_shoulder = stlPlot(obj.v_shoulder, obj.f_shoulder, name, 'FaceColor', obj.body_color, 'Parent', obj.g_shoulder);
            obj.h_upperarm = stlPlot(obj.v_upperarm, obj.f_upperarm, name, 'FaceColor', obj.body_color, 'Parent', obj.g_upperarm);
            obj.h_forearm = stlPlot(obj.v_forearm, obj.f_forearm, name, 'FaceColor', obj.body_color, 'Parent', obj.g_forearm);
            obj.h_wrist1 = stlPlot(obj.v_wrist1, obj.f_wrist1, name, 'FaceColor', obj.body_color, 'Parent', obj.g_wrist1);
            obj.h_wrist2 = stlPlot(obj.v_wrist2, obj.f_wrist2, name, 'FaceColor', obj.body_color, 'Parent', obj.g_wrist2);
            obj.h_wrist3 = stlPlot(obj.v_wrist3, obj.f_wrist3, name, 'FaceColor', obj.body_color, 'Parent', obj.g_wrist3);
            obj.h_tool = stlPlot(obj.v_tool, obj.f_tool, name, 'FaceColor', obj.body_color, 'Parent', obj.g_tool);
            
            obj.h_all_links = [obj.h_base
                               obj.h_shoulder
                               obj.h_upperarm
                               obj.h_forearm
                               obj.h_wrist1
                               obj.h_wrist2
                               obj.h_wrist3
                               obj.h_tool];
        end

    end
    methods
        function obj = UR5Display(varargin)
            if nargin > 0
                obj.ws_figure = varargin{1};
                set(obj.ws_figure, 'KeyPressFcn', @(src, event) obj.key_press_cb(src, event));
%                 set(obj.ws_figure, 'FigureDelete', @(src, event) obj.delete(src, event));
            end
            
            
            mesh_base_dir = fullfile(get_this_dir(), 'meshes');
            [obj.v_base, obj.f_base, obj.n_base, ~] = stlReadBinary(fullfile(mesh_base_dir, 'base.stl'));
            [obj.v_shoulder, obj.f_shoulder, obj.n_shoulder, ~] = stlReadBinary(fullfile(mesh_base_dir, 'shoulder.stl'));
            [obj.v_upperarm, obj.f_upperarm, obj.n_upperarm, ~] = stlReadBinary(fullfile(mesh_base_dir, 'upperarm.stl'));
            [obj.v_forearm, obj.f_forearm, obj.n_forearm, ~] = stlReadBinary(fullfile(mesh_base_dir, 'forearm.stl'));
            [obj.v_wrist1, obj.f_wrist1, obj.n_wrist1, ~] = stlReadBinary(fullfile(mesh_base_dir, 'wrist1.stl'));
            [obj.v_wrist2, obj.f_wrist2, obj.n_wrist2, ~] = stlReadBinary(fullfile(mesh_base_dir, 'wrist2.stl'));
            [obj.v_wrist3, obj.f_wrist3, obj.n_wrist3, ~] = stlReadBinary(fullfile(mesh_base_dir, 'wrist3.stl'));
            [obj.v_tool, obj.f_tool, obj.n_tool, ~] = stlReadBinary(fullfile(mesh_base_dir, 'tool.stl'));

            obj.ur5_kin = UR5Kinematics();
            obj.offset_1 = 0.135;
            obj.offset_2 = 0.425;
            obj.offset_3 = 0.39225;
            obj.offset_5 = 0.09465;
            obj.offset_6 = 0.0823;
            obj.offset_7 = 0.015;
            
            obj.offset_tool = obj.offset_6;
            
            obj.body_color = [0.2000    0.4000    1.0000];
            
            obj.ur5_base = hgtransform;
            obj.ur5_base.Matrix = eye(4);
            obj.ur5_group = hggroup;
            obj.ur5_group.Parent = obj.ur5_base;


            obj.g_base = hgtransform;
            obj.g_base.Parent = obj.ur5_group;

            obj.g_shoulder = hgtransform;
            obj.g_shoulder.Parent = obj.g_base;

            obj.g_upperarm = hgtransform;
            obj.g_upperarm.Parent = obj.g_base;

            obj.g_forearm = hgtransform;
            obj.g_forearm.Parent = obj.g_base;

            obj.g_wrist1 = hgtransform;
            obj.g_wrist1.Parent = obj.g_base;

            obj.g_wrist2 = hgtransform;
            obj.g_wrist2.Parent = obj.g_base;

            obj.g_wrist3 = hgtransform;
            obj.g_wrist3.Parent = obj.g_base;
            
            obj.g_tool = hgtransform;
            obj.g_tool.Parent = obj.g_base;
            
            obj.precomp_matrices{1} = makehgtform('xrotate', -pi/2) *  makehgtform('zrotate', pi); 
            obj.precomp_matrices{2} = makehgtform('xrotate', pi/2) * makehgtform('yrotate', -pi/2) * makehgtform('translate', [0 obj.offset_1 -obj.offset_2]);
            obj.precomp_matrices{3} = makehgtform('xrotate', pi/2) * makehgtform('yrotate', -pi/2) * makehgtform('translate', [0 obj.offset_7 -obj.offset_3]);
            obj.precomp_matrices{4} = makehgtform('translate', [0 -obj.offset_5 0]);
            obj.precomp_matrices{5} = makehgtform('xrotate', pi/2) * makehgtform('translate', [0 0 -obj.offset_5]);
            obj.precomp_matrices{6} = makehgtform('xrotate', pi/2) * makehgtform('translate', [0 -obj.offset_6 0]);
            obj.precomp_matrices{7} = makehgtform('translate', [0 obj.offset_tool 0]);

            obj.control_mode = UR5Display.JOINT_MODE;
            obj.q_display = zeros(1, 6);
            obj.q_delta = 0.05;
            obj.draw_configuration(obj.q_display);

            
            obj.h_all_links = [];
            obj.init_display();

        end
        function delete(obj, varargin)
            delete(obj.h_all_links);
        end

        function set_color(obj, color_val)
           for k = 1 : length(obj.h_all_links)
              obj.h_all_links(k).FaceColor = color_val;
           end
        end
        function set_base_pose(obj, position, orientation)
            obj.ur5_base.Matrix = makehgtform('xrotate', orientation(1)) * ...
                                  makehgtform('yrotate', orientation(2)) * ...
                                  makehgtform('zrotate', orientation(3)) * ...
                                  makehgtform('translate', position);
        end
        function draw_configuration(obj, q)
            if true == any(abs(q) > pi)
                warning('UR5 Display : q_new is overlimits');
            end
            obj.q_display = q;
            result = obj.ur5_kin.collision_balls(q);
            T1 = result.transform_matrices.T1;
            T2 = result.transform_matrices.T2;
            T3 = result.transform_matrices.T3;
            T4 = result.transform_matrices.T4;
            T5 = result.transform_matrices.T5;
            T6 = result.transform_matrices.T6;

            obj.g_base.Matrix = eye(4);
            obj.g_shoulder.Matrix = T1 * obj.precomp_matrices{1}; 
            obj.g_upperarm.Matrix = T2 * obj.precomp_matrices{2};
            obj.g_forearm.Matrix = T3 * obj.precomp_matrices{3};
            obj.g_wrist1.Matrix = T4 * obj.precomp_matrices{4};
            obj.g_wrist2.Matrix = T5 * obj.precomp_matrices{5};
            obj.g_wrist3.Matrix = T6 * obj.precomp_matrices{6};
            obj.g_tool.Matrix = obj.g_wrist3.Matrix * obj.precomp_matrices{7};
        end

    end
end

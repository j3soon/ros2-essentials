classdef InterbotixRobotXSCore < handle
    % InterbotixRobotXSCore Class that interfaces with the xs_sdk node ROS interfaces
    %   Handles subscribers, publishers, and service clients to interact with 
    %   the corresponding features of the xs_sdk

    properties
        % ros_ip - IP address that ROS should connect to
        ros_ip string = "localhost"
        
        % joint_states - Name of the joint_states topic
        joint_states struct = struct
        
        % robot_name - Namespace of the robot
        robot_name string = ""
        
        % srv_set_op_modes - Service client for /set_operating_modes
        srv_set_op_modes
        
        % srv_set_pids - Service client for /set_motor_pid_gains
        srv_set_pids
        
        % srv_set_reg - Service client for /set_motor_registers
        srv_set_reg
        
        % srv_get_reg - Service client for /get_motor_registers
        srv_get_reg
        
        % srv_get_info - Service client for /get_robot_info
        srv_get_info
        
        % srv_torque - Service client for /torque_enable
        srv_torque
        
        % srv_reboot - Service client for /reboot_motors
        srv_reboot
        
        % pub_group - Publisher for /commands/joint_group
        pub_group
        
        % pub_single - Publisher for /commands/joint_single
        pub_single
        
        % pub_traj - Publisher for /commands/joint_group
        pub_traj
        
        % sub_joint_states - Subscriber to the specified joint_states topic
        sub_joint_states
        
        % js_index_map - maps of joint names to their index in the joint_states 
        % array
        js_index_map
    end
    
    methods (Access = public)
        function obj = InterbotixRobotXSCore(robot_model, robot_name, init_node, joint_state_topic, ros_ip)
        % Constructor for the InterbotixRobotXSCore object
        %   Sets up subscribers, publishers, service clients, and js_index_map
            arguments
                robot_model       string 
                robot_name        string = ""
                init_node         {mustBeNumericOrLogical} = true
                joint_state_topic string = "joint_states"
                ros_ip            string = getenv("ROS_IP")
            end
            obj.joint_states = [];
            obj.ros_ip = ros_ip;
            
            % set robot name to robot_model if not already set
            if robot_name == ""
                obj.robot_name = robot_model;
            end
            if init_node
                rosinit(obj.ros_ip,'NodeName', ...
                    obj.robot_name + "_robot_manipulation");
            end

            % create ROS service clients and wait for servers
            obj.srv_set_op_modes = rossvcclient(...
                "/" + obj.robot_name + "/set_operating_modes", ...
                "interbotix_xs_msgs/OperatingModes");
            obj.srv_set_pids = rossvcclient(...
                "/" + obj.robot_name + "/set_motor_pid_gains", ...
                "interbotix_xs_msgs/MotorGains");
            obj.srv_set_reg = rossvcclient(...
                "/" + obj.robot_name + "/set_motor_registers", ...
                "interbotix_xs_msgs/RegisterValues");
            obj.srv_get_reg = rossvcclient(...
                "/" + obj.robot_name + "/get_motor_registers", ...
                "interbotix_xs_msgs/RegisterValues");
            obj.srv_get_info = rossvcclient(...
                "/" + obj.robot_name + "/get_robot_info", ...
                "interbotix_xs_msgs/RobotInfo");
            obj.srv_torque = rossvcclient(...
                "/" + obj.robot_name + "/torque_enable", ...
                "interbotix_xs_msgs/TorqueEnable");
            obj.srv_reboot = rossvcclient(...
                "/" + obj.robot_name + "/reboot_motors", ...
                "interbotix_xs_msgs/Reboot");
            try
                waitForServer(obj.srv_set_op_modes, "Timeout", 5.0)
                waitForServer(obj.srv_set_pids)
                waitForServer(obj.srv_set_reg)
                waitForServer(obj.srv_get_reg)
                waitForServer(obj.srv_get_info)
                waitForServer(obj.srv_torque)
                waitForServer(obj.srv_reboot)
            catch ME
                error("\nThe robot '%s' is not discoverable. Did you enter the correct robot_name parameter?\n", robot_model)
            end
            % create publishers and subscribers
            obj.pub_group  = rospublisher( ...
                "/" + obj.robot_name + "/commands/joint_group", ...
                "interbotix_xs_msgs/JointGroupCommand");
            obj.pub_single = rospublisher( ...
                "/" + obj.robot_name + "/commands/joint_single", ...
                "interbotix_xs_msgs/JointSingleCommand");
            obj.pub_traj = rospublisher( ...
                "/" + obj.robot_name + "/commands/joint_trajectory", ...
                "interbotix_xs_msgs/JointTrajectoryCommand");
            obj.sub_joint_states = rossubscriber( ...
                "/" + obj.robot_name + "/" + joint_state_topic, ...
                @obj.joint_state_cb, ...
                "DataFormat", "struct");
            % wait to receive joint_states data
            while isempty(obj.joint_states)
                pause(0.01);
            end
            % the joint states index map maps joint names to their index in 
            % the joint_states array
            obj.js_index_map = containers.Map( ...
                obj.joint_states.Name,         ...
                1:length(obj.joint_states.Name));
            pause(0.5)
            fprintf("\nRobot Name: %s\nRobot Model: %s\n", obj.robot_name, robot_model)
            fprintf("Initialized InterbotixRobotXSCore!\n")
        end

        function robot_set_operating_modes(obj, cmd_type, name, mode, profile_type, profile_velocity, profile_acceleration)
        % robot_set_operating_modes Set the operating mode for either a single 
        %   motor or a group of motors
            arguments
                obj InterbotixRobotXSCore
                
                % cmd_type - can be "group" for a group of motors or "single" 
                % for a single motor
                cmd_type string
                
                % name - group name if cmd_type is 'group' or the motor name if 
                % cmd_type is 'single'
                name string
                
                % mode - desired operatinge mode like "position" or "velocity". 
                % See the OperatingModes Service description for all choices
                mode string
                
                % profile_type - can be "time" or "velocity". See the 
                % OperatingModes Service description for details
                profile_type string = "velocity"
                
                % profile_velocity - passthrough to the Profile_Velocity 
                % register. See the OperatingModes Service description for 
                % details
                profile_velocity double = 0.0
                
                % profile_acceleration - passthrough to the 
                % Profile_Acceleration register. See the OperatingModes Service 
                % description for details
                profile_acceleration double = 0.0
            end
            srv = rosmessage("interbotix_xs_msgs/OperatingModes");
            srv.CmdType = cmd_type;
            srv.Name = name;
            srv.Mode = mode;
            srv.ProfileType = profile_type;
            srv.ProfileVelocity = profile_velocity;
            srv.ProfileAcceleration = profile_acceleration;
            obj.srv_set_op_modes.call(srv);
        end

        function robot_set_motor_pid_gains(obj, cmd_type, name, kp_pos, ki_pos, kd_pos, k1, k2, kp_vel, ki_vel)
        % robot_set_motor_pid_gains Set the internal PID gains for either a 
        %   single motor or a group of motors
            arguments
                obj InterbotixRobotXSCore
                
                % cmd_type - can be "group" for a group of motors or "single" 
                % for a single motor
                cmd_type string
                
                % name - group name if cmd_type is 'group' or the motor name if 
                % cmd_type is 'single'
                name string
                
                % kp_pos - passthrough to the Position_P_Gain register. See 
                % the MotorGains Service description for details
                kp_pos double = 0.0
                
                % ki_pos - passthrough to the Position_I_Gain register. See 
                % the MotorGains Service description for details
                ki_pos double = 0.0
                
                % kd_pos - passthrough to the Position_D_Gain register. See 
                % the MotorGains Service description for details
                kd_pos double = 0.0
                
                % k1 - passthrough to the Feedforward_1st_Gain register. See 
                % the MotorGains Service description for details
                k1 double = 0.0
                
                % k2 - passthrough to the Feedforward_2nd_Gain register. See 
                % the MotorGains Service description for details
                k2 double = 0.0
                
                % kp_vel - passthrough to the Velocity_P_Gain register. See 
                % the MotorGains Service description for details
                kp_vel double = 100.0
                
                % ki_vel - passthrough to the Velocity_I_Gain register. See 
                % the MotorGains Service description for details
                ki_vel double = 1920.0
            end
            srv = rosmessage("interbotix_xs_msgs/MotorGains");
            srv.CmdType = cmd_type;
            srv.Name = name;
            srv.Mode = mode;
            srv.KpPos = kp_pos;
            srv.KiPos = ki_pos;
            srv.KdPos = kd_pos;
            srv.K1 = k1;
            srv.K2 = k2;
            srv.KpVel = kp_vel;
            srv.KiVel = ki_vel;
            obj.srv_set_pids.call(srv);
        end

        function response = robot_set_motor_registers(obj, cmd_type, name, reg, value)
        % robot_set_motor_registers Set the desired register for either a 
        %   single motor or a group of motors
        % returns response - list of register values
            arguments
                obj InterbotixRobotXSCore
                
                % cmd_type - can be "group" for a group of motors or "single" 
                % for a single motor
                cmd_type string
                
                % name - group name if cmd_type is 'group' or the motor name if 
                % cmd_type is 'single'
                name string
                
                % reg - desired register name
                reg string
                
                % value - desired value for the above register
                value string
            end
            srv = rosmessage("interbotix_xs_msgs/RegisterValues");
            srv.CmdType = cmd_type;
            srv.Name = name;
            srv.Reg = reg;
            srv.Value = value;
            response = obj.srv_set_reg.call(srv);
        end

        function response = robot_get_motor_registers(obj, cmd_type, name, reg)
        % robot_get_motor_registers Get the desired register value from either 
        %   a single motor or a group of motors
            arguments
                obj InterbotixRobotXSCore
                
                % cmd_type - can be "group" for a group of motors or "single" 
                % for a single motor
                cmd_type string
                
                % name - group name if cmd_type is 'group' or the motor name if 
                % cmd_type is 'single'
                name string
                
                % reg - desired register name
                reg string
            end
            srv = rosmessage("interbotix_xs_msgs/RegisterValues");
            srv.CmdType = cmd_type;
            srv.Name = name;
            srv.Reg = reg;
            response = obj.srv_get_reg.call(srv);
        end
        
        function response = robot_get_robot_info(obj, cmd_type, name)
        % robot_get_robot_info Get information about the robot - mostly joint 
        %   limit data
        % 
        % returns response - an object with the same structure as a RobotInfo 
        %                    Service description
            arguments
                obj InterbotixRobotXSCore
                
                % cmd_type - can be "group" for a group of motors or "single" 
                % for a single motor
                cmd_type string
                
                % name - group name if cmd_type is 'group' or the motor name if 
                % cmd_type is 'single'
                name string
            end
            srv = rosmessage("interbotix_xs_msgs/RobotInfo");
            srv.CmdType = cmd_type;
            srv.Name = name;
            response = obj.srv_get_info.call(srv);
        end

        function robot_torque_enable(obj, cmd_type, name, enable)
        % robot_torque_enable Torque a single motor or a group of motors to be 
        %   on or off
            arguments
                obj InterbotixRobotXSCore
                
                % cmd_type - can be "group" for a group of motors or "single" 
                % for a single motor
                cmd_type string
                
                % name - group name if cmd_type is 'group' or the motor name if 
                % cmd_type is 'single'
                name string
                
                % enable - True to torque on or False to torque off
                enable boolean
            end
            srv = rosmessage("interbotix_xs_msgs/TorqueEnable");
            srv.CmdType = cmd_type;
            srv.Name = name;
            srv.Enable = enable;
            obj.srv_torque.call(srv);
        end

        function robot_reboot_motors(obj, cmd_type, name, enable, smart_reboot)
        % robot_reboot_motors Reboot a single motor or a group of motors if 
        %   they are in an error state
            arguments
                obj InterbotixRobotXSCore
                
                % cmd_type - can be "group" for a group of motors or "single" 
                % for a single motor
                cmd_type string
                
                % name - group name if cmd_type is 'group' or the motor name if 
                % cmd_type is 'single'
                name string
                
                % enable - True to torque on or False to leave torqued off 
                % after rebooting
                enable boolean
                
                % smart_reboot - if 'cmd_type' is set to 'group', setting this 
                % to True will only reboot
                smart_reboot boolean = false
            end
            srv = rosmessage("interbotix_xs_msgs/Reboot");
            srv.CmdType = cmd_type;
            srv.Name = name;
            srv.Enable = enable;
            srv.SmartReboot = smart_reboot;
            obj.srv_reboot.call(srv);
        end

        function robot_write_commands(obj, group_name, commands)
        % robot_write_commands Command a group of motors (refer to the 
        %   JointGroupCommand Message description for more info)
            arguments
                obj InterbotixRobotXSCore
                
                % group_name - the group name of the motors to command
                group_name string
                
                % commands - desired list of commands
                commands
            end
            msg = rosmessage("interbotix_xs_msgs/JointGroupCommand");
            msg.Name = group_name;
            msg.Cmd = commands;
            obj.pub_group.send(msg);
        end

        function robot_write_joint_command(obj, joint_name, command)
        % robot_write_joint_command Command a single motor (refer to the 
        %   JointSingleCommand Message description for more info)
            arguments
                obj InterbotixRobotXSCore
                
                % joint_name - the name of the motor to command
                joint_name  string
                
                % command - desired command
                command
            end
            msg = rosmessage("interbotix_xs_msgs/JointSingleCommand");
            msg.Name = joint_name;
            msg.Cmd = command;
            obj.pub_single.send(msg);
        end

        function robot_write_trajectory(obj, cmd_type, name, type, raw_traj)
        % robot_write_trajectory Command a trajectory of positions or 
        % velocities to a single motor or a group of motors
            arguments
                obj InterbotixRobotXSCore
                
                % cmd_type - can be "group" for a group of motors or "single" 
                % for a single motor
                cmd_type string
                
                % name - group name if cmd_type is 'group' or the motor name if 
                % cmd_type is 'single'
                name string
                
                % type - "position" if the trajectory is a array of positions [rad]; 
                % "velocity" if the trajectory is a array of velocities [rad/s]
                type string
                
                % raw_traj - array of dictionaries where each dictionary is made 
                % up of a double / array of double pairs. the 'key' is the 
                % desired time [sec] from start that the 'value' (array of 
                % doubles) should be executed.
                raw_traj containers.Map

            end
            % Map is unordered - sort by times
            all_traj_times = sort(cell2mat(raw_traj.keys()));
            traj = rosmessage("trajectory_msgs/JointTrajectory");
            for i=1:length(all_traj_times)
                point_time = all_traj_times(i);
                points = raw_traj(point_time);
                point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
                if (type == "position")
                    point.Positions = points;
                elseif (type == "velocity")
                    point.Velocities = points;
                end
                point.TimeFromStart = rosduration(point_time);
                traj.Points(end+1) = point;
            end
            traj_cmd = rosmessage("interbotix_xs_msgs/JointTrajectoryCommand");
            traj_cmd.CmdType = cmd_type;
            traj_cmd.Name = name;
            traj_cmd.Traj = traj;
            obj.pub_traj.send(traj_cmd);
            pause(all_traj_times(end))
        end

        function joint_states = robot_get_joint_states(obj)
        % robot_get_joint_states Get the current joint states (position, velocity, effort) of all Dynamixel motors
        % 
        % returns joint_states - JointState ROS message. Refer to online 
        %   documentation to see its structure
            arguments
                obj InterbotixRobotXSCore
            end
            joint_states = obj.joint_states;
        end

        function joint_info = robot_get_single_joint_state(obj, name)
        % robot_get_single_joint_state Get a single joint state for the specified Dynamixel motor
        % 
        % returns joint_info - struct with 3 keys: "Position", "Velocity", 
        %   and "Effort". Units are rad, rad/s, and mA
            arguments
                obj InterbotixRobotXSCore

                % name - desired motor name for which to get the joint state
                name string
            end
            js = obj.joint_states;
            cellnames = strfind(js.Name, name);
            for i=1:length(cellnames)
                if cellnames{i} == 1
                    break
                end
            end
            joint_info = struct;
            joint_info.Position = js.Position(i);
            if isempty(js.Effort)
                joint_info.Velocity = zeros(length(js.Position(i)));
                joint_info.Effort = zeros(length(js.Position(i)));
            else
                joint_info.Velocity = js.Velocity(i);
                joint_info.Effort   = js.Effort(i);
            end
        end

        function joint_state_cb(obj, ~, msg)
        % joint_state_cb ROS Subscriber Callback function to get the latest JointState message
            obj.joint_states = msg;
        end
        
        function rotm = eul_to_rotm(obj, eul)
            alpha = eul(1);
            beta  = eul(2);
            gamma = eul(3);
            Rx = [1, 0,           0
                  0, cos(alpha), -sin(alpha)
                  0, sin(alpha),  cos(alpha)];

            Ry = [cos(beta),  0, sin(beta)
                  0,          1, 0
                 -sin(beta),  0, cos(beta)];

            Rz = [cos(gamma), -sin(gamma), 0
                  sin(gamma),  cos(gamma), 0
                  0,           0,          1];
            rotm = Rz * (Ry * Rx);
        end

        function eul = rotm_to_eul(obj, rotm)
            sy = sqrt(rotm(1,1) * rotm(1,1) + rotm(2,1) * rotm(2,1));
            if sy < 1e-6
                eul(1) = atan2(-rotm(2,3), rotm(2,2));
                eul(2) = atan2(-rotm(3,1), sy);
                eul(3) = 0;
            else
                eul(1) = atan2(rotm(3,2), rotm(3,3));
                eul(2) = atan2(-rotm(3,1), sy);
                eul(3) = atan2(rotm(2,1), rotm(1,1));
            end
        end
    end
end

classdef InterbotixArmXSInterface < handle
    % Definition of the Interbotix Arm Module
    properties
        % core - Reference to the InterbotixRobotXSCore class containing the 
        % internal ROS plumbing that drives the MATLAB API
        core InterbotixRobotXSCore
        
        % group_info - Struct containing information for the arm's group
        group_info
        
        % robot_des - Struct containing the modern robotics description used 
        % for kinematic calculations
        robot_des
        
        % initial_guesses - Array of initial guesses to give to the inverse 
        % kinematics solver
        initial_guesses
        
        % moving_time - time [s] it should take for all joints in the arm to 
        % complete one move
        moving_time
        
        % accel_time - time [s] it should take for all joints in the arm to 
        % accelerate/decelerate to/from max speed
        accel_time
        
        % group_name - joint group name that contains the 'arm' joints as 
        % defined in the % 'motor_config' yaml file; typically, this is 'arm'
        group_name
        
        % info_index_map - Map of joint info to their index in the group_info 
        % array
        info_index_map
        
        % joint_commands - An array containing the previous commands given to 
        % the joints
        joint_commands (:,1)
        
        % rev - 2*pi
        rev (1,1) double = 2*pi
        
        % T_sb - The space (s) to body (b) transformation matrix in SE(3)
        T_sb (4,4) double
    end
    
    methods
        function obj = InterbotixArmXSInterface(core, robot_model, group_name, opts)
            % Constructor for the InterbotixArmXSInterface object
            arguments
                
                % core - reference to the InterbotixRobotXSCore object 
                % containing the internal ROS plumbing that drives the MATLAB API
                core InterbotixRobotXSCore
                
                % robot_model - Interbotix Arm model (ex. 'wx200' or 'vx300s')
                robot_model string = ""
                
                % group_name - joint group name that contains the 'arm' joints 
                % as defined in % the 'motor_config' yaml file; typically, this 
                % is 'arm'
                group_name string = ""
                
                % moving_time - time [s] it should take for all joints in the 
                % arm to complete one move
                opts.moving_time double = 2.0
                
                % accel_time - time [s] it should take for all joints in the 
                % arm to accelerate/decelerate to/from max speed
                opts.accel_time double = 3.0
            end
            obj.core = core;

            % get the robot information
            srv = rosmessage("interbotix_xs_msgs/RobotInfoRequest");
            srv.CmdType = "group";
            srv.Name = group_name;
            obj.group_info = obj.core.srv_get_info.call(srv);

            % Inform the user that they must be using time profile and position mode
            if (obj.group_info.ProfileType ~= "time")
                disp("Please set the group's 'profile type' to 'time'.");
            end
            if (obj.group_info.Mode ~= "position")
                disp("Please set the group's 'operating mode' to 'position'.");
            end

            % get the robot's mr_description
            obj.robot_des = mr_descriptions.(robot_model);

            % build the initial guesses
            obj.initial_guesses = zeros(obj.group_info.NumJoints,3);
            obj.initial_guesses(2,1) = deg2rad(-120);
            obj.initial_guesses(3,1) = deg2rad(120);

            obj.moving_time = opts.moving_time;
            obj.accel_time = opts.accel_time;
            obj.group_name = group_name;
            obj.joint_commands = zeros(obj.group_info.NumJoints,1);

            % build the joint_commands array from the current positions
            for i=1:obj.group_info.NumJoints
                name = obj.group_info.JointNames(i);
                obj.joint_commands(i) = ...
                    obj.core.joint_states.Position( ...
                        obj.core.js_index_map(name{1}));
            end
            
            % using the current joint positions, build the base to end effector 
            %   transform matrix
            obj.T_sb = FKinSpace( ...
                obj.robot_des.M, obj.robot_des.Slist, obj.joint_commands);
            
            % set the update trajectory timing parameters from the given args
            obj.set_trajectory_time( ...
                moving_time=opts.moving_time, accel_time=opts.accel_time);
            
            % build the info_index_map
            obj.info_index_map = containers.Map( ...
                obj.group_info.JointNames, 1:obj.group_info.NumJoints);
            
            fprintf( ...
                "\nArm Group Name: %s\nMoving Time: %.2f seconds\nAcceleration Time: %.2f seconds\nDrive Mode: Time-Based-Profile\n", ...
                group_name, opts.moving_time, opts.accel_time)
            
            fprintf("Initialized InterbotixArmXSInterface!\n")
        end
        
        function publish_positions(obj, positions, opts)
        % publish_positions Helper function to publish joint positions and 
        %   block if necessary
            arguments
                obj InterbotixArmXSInterface
                
                % positions - desired joint positions
                positions (1,:) double
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []
                
                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
                
                % blocking - whether the function should wait to return control 
                % to the user until the robot finishes moving
                opts.blocking {mustBeNumericOrLogical} = true
            end
            
            % update trajectory timing parameters
            obj.set_trajectory_time( ...
                moving_time=opts.moving_time, accel_time=opts.accel_time);
            obj.joint_commands = positions;
            msg = rosmessage("interbotix_xs_msgs/JointGroupCommand");
            msg.Name = obj.group_name;
            msg.Cmd = obj.joint_commands;
            obj.core.pub_group.send(msg);

            % pause for the moving_time
            if opts.blocking
                pause(obj.moving_time);
            end

            % update the transform
            obj.T_sb = FKinSpace(...
                obj.robot_des.M, obj.robot_des.Slist, obj.joint_commands);
        end

        function set_trajectory_time(obj, opts)
        % set_trajectory_time Helper function to command the 'Profile_Velocity' 
        %   and 'Profile_Acceleration' motor registers
            arguments
                obj InterbotixArmXSInterface
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []
                
                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
            end

            % update moving_time
            if (~isempty(opts.moving_time) && opts.moving_time ~= obj.moving_time)
                obj.moving_time = opts.moving_time;
                srv = rosmessage("interbotix_xs_msgs/RegisterValuesRequest");
                srv.CmdType = "group";
                srv.Name = obj.group_name;
                srv.Reg = "Profile_Velocity";
                srv.Value = fix(opts.moving_time * 1000);
                obj.core.srv_set_reg.call(srv);
            end
            
            % update accel_time
            if (~isempty(opts.accel_time) && opts.accel_time ~= obj.accel_time)
                obj.accel_time = opts.accel_time;
                srv = rosmessage("interbotix_xs_msgs/RegisterValuesRequest");
                srv.CmdType = "group";
                srv.Name = obj.group_name;
                srv.Reg = "Profile_Acceleration";
                srv.Value = fix(opts.accel_time * 1000);
                obj.core.srv_set_reg.call(srv);
            end
        end

        function limits_ok = check_joint_limits(obj, positions)
        % check_joint_limits Helper function to check to make sure the desired 
        %   arm group's joint positions are all within their respective joint 
        %   limits
        % 
        % returns limits_ok - True if all positions are within limits 
        %                     False otherwise
            arguments
                obj InterbotixArmXSInterface

                % the positions [rad] to check
                positions double
            end

            % build an array of positions and speeds for each joint command
            theta_list = arrayfun(@(elem)(fix(elem * 1000)/1000.0), positions);
            speed_list = arrayfun( ...
                @(goal,current)(abs(goal - current)/obj.moving_time), ...
                repmat(obj.joint_commands,1,size(theta_list,2)), ...
                obj.joint_commands);
            
            % check position and velocity limit of each joint
            for i=1:obj.group_info.NumJoints
                % check position upper and lower limits
                if ~((obj.group_info.JointLowerLimits(i) <= theta_list(i)) && ...
                     (obj.group_info.JointUpperLimits(i) >= theta_list(i)))
                    limits_ok = false;
                    return
                end

                % check velocity limits
                if (speed_list(i) > obj.group_info.JointVelocityLimits(i))
                    limits_ok = false;
                    return
                end
            end

            % return true if no limits are violated
            limits_ok = true;
            return
        end

        function limit_ok = check_single_joint_limit(obj, joint_name, position)
        % check_single_joint_limit Helper function to check to make sure a 
        % desired position for a given joint is within its limits
            arguments
                obj InterbotixArmXSInterface
                
                % joint_name - desired joint name
                joint_name string
                
                % position - desired joint position [rad]
                position double
            end

            % get position and speed for joint command
            theta = fix(position * 1000)/1000.0;
            speed = abs(theta - obj.joint_commands(obj.info_index_map(joint_name)))/(obj.moving_time);

            % get limits for the selected joint
            ll = obj.group_info.JointLowerLimits(obj.info_index_map(joint_name));
            ul = obj.group_info.JointUpperLimits(obj.info_index_map(joint_name));
            vl = obj.group_info.JointVelocityLimits(obj.info_index_map(joint_name));
            
            % check upper and lower joint position limits
            if ~((ll <= theta) && (theta <= ul))
                limit_ok = false;
                return
            end

            % check joint velocity limit
            if (speed > vl)
                limit_ok = false;
                return 
            end

            % return true if no limits are violated
            limit_ok = true;
            return 
        end

        function commanded = set_joint_positions( ...
            obj, joint_positions, opts)
        % set_joint_positions Command positions to the arm joints
            arguments
                obj InterbotixArmXSInterface
                
                % joint_positions - desired joint positions [rad]
                joint_positions (:,1) double
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []
                
                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
                
                % blocking - whether the function should wait to return control 
                % to the user until the robot finishes moving
                opts.blocking {mustBeNumericOrLogical} = 1
            end

            % check to make sure array is sized properly
            if size(joint_positions,1)~=obj.group_info.NumJoints
                if size(joint_positions,2)==obj.group_info.NumJoints
                    joint_positions = joint_positions';
                else
                    % throw error is incorrect length
                    error( ...
                        "You must provide an array of length %i.", ...
                        obj.group_info.NumJoints)
                end
            end
            commanded = true;
            
            % if joints limits are not violated, publish the position commands
            if (obj.check_joint_limits(joint_positions))
                obj.publish_positions( ...
                    joint_positions, moving_time=opts.moving_time, ...
                    accel_time=opts.accel_time, blocking=opts.blocking);
            else
                disp("Joint limits violated. Unable to set joint positions.")
                commanded = false;
                return
            end
        end

        function go_to_home_pose(obj, opts)
        % go_to_home_pose Command the arm to go to its Home pose
            arguments
                obj InterbotixArmXSInterface
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []
                
                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
                
                % blocking - whether the function should wait to return control 
                % to the user until the robot finishes moving
                opts.blocking {mustBeNumericOrLogical} = 1
            end

            % publish home position
            obj.publish_positions( ...
                zeros(obj.group_info.NumJoints,1), ...
                moving_time=opts.moving_time, ...
                accel_time=opts.accel_time, blocking=opts.blocking);
        end

        function go_to_sleep_pose(obj, opts)
        % go_to_sleep_pose Command the arm to go to its Sleep pose
            arguments
                obj InterbotixArmXSInterface
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []
                
                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
                
                % blocking - whether the function should wait to return control 
                % to the user until the robot finishes moving
                opts.blocking {mustBeNumericOrLogical} = 1
            end

            % publish sleep position
            obj.publish_positions( ...
                obj.group_info.JointSleepPositions, ...
                moving_time=opts.moving_time, ...
                accel_time=opts.accel_time, blocking=opts.blocking);

        end

        function return_safely(obj, opts)
        % return_safely Command the arm to go to its Sleep pose safely
            arguments
                obj InterbotixArmXSInterface
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []
                
                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
                
                % blocking - whether the function should wait to return control 
                % to the  user until the robot finishes moving
                opts.blocking {mustBeNumericOrLogical} = 1
            end

            % publish home position
            obj.go_to_home_pose( ...
                moving_time=opts.moving_time, ...
                accel_time=opts.accel_time, ...
                blocking=opts.blocking);

            % publish sleep position
            obj.go_to_sleep_pose( ...
                moving_time=opts.moving_time, ...
                accel_time=opts.accel_time, ...
                blocking=opts.blocking);
        end

        function commanded = set_single_joint_position( ...
            obj, joint_name, position, opts)
        % set_single_joint_position Command a single joint to a desired position
            arguments
                obj InterbotixArmXSInterface
                
                % joint_name - name of the joint to control
                joint_name string
                
                % position - desired position [rad]
                position double
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []
                
                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
                
                % blocking - whether the function should wait to return control 
                % to the user until the robot finishes moving
                opts.blocking {mustBeNumericOrLogical} = 1
            end
            commanded = true;

            % check that the command will not limit the joint limits
            if ~obj.check_single_joint_limit(joint_name, position)
                disp("Joint limit violated. Unable to set joint position.")
                commanded = false;
                return
            end

            % update trajectory timing parameters 
            obj.set_trajectory_time( ...
                moving_time=opts.moving_time, accel_time=opts.accel_time);

            % get the last joint command for this joint - should be an accurate 
            %   current position reading
            obj.joint_commands(obj.core.js_index_map(joint_name)) = position;

            % send the joint command
            single_command = rosmessage("interbotix_xs_msgs/JointSingleCommand");
            single_command.Name = joint_name;
            single_command.Cmd = position;
            obj.core.pub_single.send(single_command);
            
            % pause for the moving_time
            if opts.blocking
                pause(obj.moving_time);
            end

            % update the transform
            obj.T_sb = FKinSpace( ...
                obj.robot_des.M, obj.robot_des.Slist, obj.joint_commands);
        end

        function [theta_list, found] = set_ee_pose_matrix( ...
            obj, T_sd, custom_guess, opts)
        % set_ee_pose_matrix Command a desired end-effector pose
        % 
        % returns theta_list - joint values needed to get the end-effector to 
        %                      the desired pose
        %         found - True if a valid solution was found; False otherwise
            arguments
                obj InterbotixArmXSInterface
                
                % T_sd - 4x4 Transformation Matrix representing the transform 
                % from the /<robot_name>/base_link frame to the 
                % /<robot_name>/ee_gripper_link frame
                T_sd (4,4) double
                
                % custom_guess - list of joint positions with which to seed the 
                % IK solver
                custom_guess double = []
                
                % execute - if True, this moves the physical robot after 
                % planning; otherwise, only planning is done
                opts.execute {mustBeNumericOrLogical} = 1
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []
                
                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
                
                % blocking - whether the function should wait to return control 
                % to the user until the robot finishes moving
                opts.blocking {mustBeNumericOrLogical} = 1
            end

            % get initial guesses
            if (isempty(custom_guess))
                guesses = obj.initial_guesses;
            else
                guesses = custom_guess;
            end

            % loop through guesses
            for i=1:size(guesses,2)
                guess = guesses(:,i);
                % find IK solutions. note that you might encounter strange joint 
                % positions if no solution is found
                [theta_list, success] = IKinSpace( ...
                    obj.robot_des.Slist, obj.robot_des.M, ...
                    T_sd, guess, 0.001, 0.001);
    
                % If a solution was found, check that no joint limits are violated
                if success
                    for x=1:size(theta_list,1)

                        % wrap joints if rotation larger than 1 revolution
                        if theta_list(x) <= -obj.rev
                            theta_list(x) = mod(theta_list(x), -obj.rev);
                        elseif theta_list(x) >= obj.rev
                            theta_list(x) = mod(theta_list(x), obj.rev);
                        end
                        if round(theta_list(x),3) < round( ...
                            obj.group_info.JointLowerLimits(x),3)
                            theta_list(x) = theta_list(x) + obj.rev;
                        elseif round(theta_list(x),3) > round( ...
                            obj.group_info.JointUpperLimits(x),3)
                            theta_list(x) = theta_list(x) - obj.rev;
                        end
                    end
                    solution_found = obj.check_joint_limits(theta_list);
                else
                    solution_found = false;
                end
    
                % if a valid solution was found and execute is true, publish the commands
                if solution_found
                    if opts.execute
                        obj.publish_positions( ...
                            theta_list, moving_time=opts.moving_time, ...
                            accel_time=opts.accel_time, blocking=opts.blocking);
                        % update the transform
                        obj.T_sb = T_sd;
                    end
                    found = true;
                    return;
                end
            end
    
            disp("No valid pose could be found")
            found = false;
            return
        end
        
        function [theta_list, success] = set_ee_pose_components( ...
                obj, pose, opts)
        % set_ee_pose_components Command a desired end-effector pose w.r.t. 
        %   the Space frame
        % 
        % return theta_list - joint values needed to get the end-effector to the desired pose
        %        success - true if a valid solution was found; false otherwise
        % 
        % Note: Do not set 'yaw' if using an arm with fewer than 6dof
            arguments
                obj InterbotixArmXSInterface
                
                % x - linear position along the X-axis of the Space frame [m]
                pose.x double = 0.0
                
                % y - linear position along the Y-axis of the Space frame [m]
                pose.y double = 0.0
                
                % z - linear position along the Z-axis of the Space frame [m]
                pose.z double = 0.0
                
                % roll - angular position around the X-axis of the Space frame [rad]
                pose.roll double = 0.0
                
                % pitch - angular position around the Y-axis of the Space frame [rad]
                pose.pitch double = 0.0
                
                % yaw - angular position around the Z-axis of the Space frame [rad]
                pose.yaw double = []
                
                % custom_guess - list of joint positions with which to seed the 
                % IK solver
                opts.custom_guess double = []
                
                % execute - if True, this moves the physical robot after planning; 
                %           otherwise, only planning is done
                opts.execute {mustBeNumericOrLogical} = 1
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time double = []

                % accel_time - duration in seconds that that robot should spend 
                % accelerating/decelerating (must be less than or equal to half 
                % the moving_time)
                opts.accel_time double = []
                
                % blocking - whether the function should wait to return control 
                % to the user until the robot finishes moving
                opts.blocking {mustBeNumericOrLogical} = 1
            end

            % if dof<6 or yaw isn't specified, calculate yaw based on x and y
            if (obj.group_info.NumJoints < 6 || ...
               (obj.group_info.NumJoints >= 6 && isempty(pose.yaw)))
               pose.yaw = atan2(pose.y,pose.x);
                if isnan(pose.yaw)
                    disp("No valid yaw found.")
                    success = false;
                    return
                end
            end

            % fill out the desired transformation matrix based on the pose 
            %   components
            T_sd = eye(4);
            T_sd(1:3,1:3) = obj.core.eul_to_rotm([pose.roll, pose.pitch, pose.yaw]);
            T_sd(1:3, 4) = [pose.x, pose.y, pose.z];

            % set the pose matrix
            [theta_list, success] = obj.set_ee_pose_matrix( ...
                T_sd, opts.custom_guess, execute=opts.execute, ...
                moving_time=opts.moving_time, accel_time=opts.accel_time, ...
                blocking=opts.blocking);
        end

        function success = set_ee_cartesian_trajectory(obj, pose, opts)
        % set_ee_cartesian_trajectory Command a desired end-effector 
        %   displacement that will follow a straight line path (when in 
        %   'position' control mode)
        % 
        % return success - true if a trajectory was successfully planned and 
        %   executed; otherwise false
        % 
        %   T_sy is a 4x4 transformation matrix representing the pose of a 
        %       virtual frame w.r.t. /<robot_name>/base_link. This virtual 
        %       frame has the exact same x, y, z, roll, and pitch of 
        %       /<robot_name>/base_link but contains the yaw of the 
        %       end-effector frame (/<robot_name>/ee_gripper_link). 
        %   Note that 'y' and 'yaw' must equal 0 if using arms with less than 6dof.
            arguments
                obj                     InterbotixArmXSInterface
                
                % x- linear displacement along the X-axis w.r.t. T_sy [m]
                pose.x                  double = 0
                
                % y- linear displacement along the Y-axis w.r.t. T_sy [m]
                pose.y                  double = 0
                
                % z- linear displacement along the Z-axis w.r.t. T_sy [m]
                pose.z                  double = 0
                
                % roll - angular displacement around the X-axis w.r.t. T_sy [rad]
                pose.roll               double = 0
                
                % pitch - angular displacement around the Y-axis w.r.t. T_sy [rad]
                pose.pitch              double = 0
                
                % yaw - angular displacement around the Z-axis w.r.t. T_sy [rad]
                pose.yaw                double = 0
                
                % moving_time - duration in seconds that the robot should move
                opts.moving_time        double = []
                
                % wp_moving_time - duration in seconds that each waypoint in 
                % the trajectory should move
                opts.wp_moving_time     double = 0.2
                
                % wp_accel_time - duration in seconds that each waypoint in the 
                % trajectory should be accelerating/decelerating (must be equal 
                % to or less than half of wp_moving_time)
                opts.wp_accel_time      double = 0.1
                
                % wp_period - duration in seconds between each waypoint
                opts.wp_period          double = 0.05
            end

            % check that y and yaw aren't specified if dof<6
            if (obj.group_info.NumJoints < 6) && (pose.y ~= 0 || pose.yaw ~= 0)
                disp("Please leave the 'y' and 'yaw' fields at '0' when working with arms that have less than 6dof.")
                success = false;
                return
            end

            % build transform to virtual frame
            rpy = obj.core.rotm_to_eul(obj.T_sb(1:3,1:3));
            T_sy = eye(4);
            T_sy(1:3,1:3) = obj.core.eul_to_rotm([0.0, 0.0, rpy(3)]);
            T_yb = TransInv(T_sy) * obj.T_sb;
            rpy(3) = 0.0;

            % update moving time if given
            if isempty(opts.moving_time)
                opts.moving_time = obj.moving_time;
            end

            % calculate the number of waypoints based on the length of the 
            %   trajectory and the period of the waypoints
            N = fix(opts.moving_time / opts.wp_period);

            % calculate the ratio of each waypoint in the trajectory
            inc = 1.0 / N;
            joint_traj = rosmessage("trajectory_msgs/JointTrajectory");
            joint_positions = obj.joint_commands;

            % loop through each waypoint in the trajectory, calculating each 
            %   point in the trajectory
            for i=1:N+2
                joint_traj_point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
                joint_traj_point.Positions = joint_positions;
                joint_traj_point.TimeFromStart = rosduration(i * opts.wp_period);
                joint_traj.Points(end+1) = joint_traj_point;

                % break out the loop if this is the last point
                if (i == N+1)
                    break
                end

                % increment the x,y,z position
                T_yb(1:3,4) = T_yb(1:3,4) + [inc * pose.x, inc * pose.y, inc * pose.z]';

                % increment the roll, pitch, and yaw
                rpy(1) = rpy(1) + inc * pose.roll;
                rpy(2) = rpy(2) + inc * pose.pitch;
                rpy(3) = rpy(3) + inc * pose.yaw;
                T_yb(1:3,1:3) = obj.core.eul_to_rotm(rpy);

                % update the frame with the next transform
                T_sd = T_sy * T_yb;
                [theta_list, success] = obj.set_ee_pose_matrix( ...
                    T_sd, joint_positions, execute=false, blocking=false);
                if success
                    joint_positions = theta_list;
                else
                    fprintf("%.1f%% of trajectory successfully planned. Trajectory will not be executed.\n", (i/fix(N) * 100));
                    break
                end
            end
    
            % if all points in the trajectory were calculated successfully, 
            %   build the trajectory message and send the commands
            if success
                % update trajectory timing parameters 
                obj.set_trajectory_time( ...
                    moving_time=opts.wp_moving_time, ...
                    accel_time=opts.wp_accel_time);

                % build and publish the joint trajectory message
                joint_traj.JointNames = obj.group_info.JointNames;
                current_positions = zeros(1,obj.group_info.NumJoints);
                for i=1:obj.group_info.NumJoints
                    name = joint_traj.JointNames(i);
                    current_positions(i) = obj.core.joint_states.Position( ...
                        obj.core.js_index_map(name{1}));
                end
                joint_traj.Points(1).Positions = current_positions;
                joint_traj.Header.Stamp = rostime("now");
                joint_traj_command = rosmessage("interbotix_xs_msgs/JointTrajectoryCommand");
                joint_traj_command.Traj = joint_traj;
                joint_traj_command.CmdType = "group";
                joint_traj_command.Name = obj.group_name;
                obj.core.pub_traj.send(joint_traj_command);

                % pause for the moving_time and the waypoint time
                pause(opts.moving_time + opts.wp_moving_time);

                % update the transform and joint_commands
                obj.T_sb = T_sd;
                obj.joint_commands = joint_positions;

                % update trajectory timing parameters 
                obj.set_trajectory_time( ...
                    moving_time=opts.moving_time, accel_time=obj.accel_time);
            end
        end

        function joint_commands = get_joint_commands(obj)
        % get_joint_commands Get the latest commanded joint positions
        % 
        % return joint_commands - array of latest commanded joint positions [rad]
            arguments
                obj InterbotixArmXSInterface
            end
            joint_commands = obj.joint_commands;
        end

        function [joint_command] = get_single_joint_command(obj, joint_name)
        % get_single_joint_command Get the latest commanded position for a 
        %   given joint
        % 
        % return joint_command - desired position [rad]
            arguments
                obj InterbotixArmXSInterface
                
                % joint_name - joint for which to get the position
                joint_name string
            end
            joint_command = obj.joint_commands(obj.info_index_map(joint_name));
        end

        function [ee_pose_command] = get_ee_pose_command(obj)
        % get_ee_pose_command Get the latest commanded end-effector pose w.r.t the Space frame
        % 
        % return ee_pose_command - Transformation matrix
            arguments
                obj InterbotixArmXSInterface
            end
            ee_pose_command = obj.T_sb;
        end

        function T_sb = get_ee_pose(obj)
        % get_ee_pose Get the actual end-effector pose w.r.t the Space frame
        % 
        % return T_sb - Transformation matrix
            arguments
                obj InterbotixArmXSInterface
            end
            joint_states = [];
            
            % get the current joint states
            for i=1:length(obj.group_info.JointNames)
                name = obj.group_info.JointNames(i);
                joint_states = [
                    joint_states'
                    obj.core.joint_states.Position(obj.core.js_index_map(name{1}))']';
            end

            % calculate the transform
            T_sb = FKinSpace(obj.robot_des.M, obj.robot_des.Slist, joint_states');
        end

        function capture_joint_positions(obj)
        % capture_joint_positions Resets self.joint_commands to be the actual 
        %   positions seen by the encoders
        % 
        %   Should be used whenever joints are torqued off, right after 
        %       torquing them on again
            arguments
                obj InterbotixArmXSInterface
            end
            obj.joint_commands = [];
            for i=1:length(obj.group_info.JointNames)
                name = obj.group_info.JointNames(i);
                obj.joint_commands = [
                    obj.joint_commands
                    obj.core.joint_states.Position(obj.core.js_index_map(name{1}))];
            end
            obj.T_sb = FKinSpace( ...
                obj.robot_des.M, obj.robot_des.Slist, obj.joint_commands);
        end

    end
end

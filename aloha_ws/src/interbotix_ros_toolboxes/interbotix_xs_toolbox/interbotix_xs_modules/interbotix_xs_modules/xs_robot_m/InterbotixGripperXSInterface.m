classdef InterbotixGripperXSInterface < handle
    
    properties
        % core - Reference to the InterbotixRobotXSCore class containing the 
        % internal ROS plumbing that drives the MATLAB API
        core InterbotixRobotXSCore
        
        % gripper_name - name of the gripper joint as defined in the 
        % 'motor_config' yaml file; typically, this is 'gripper'
        gripper_name string
        
        % gripper_moving - whether or not the gripper is moving
        gripper_moving {islogical}
        
        % gripper_command - JointSingleCommand message defining the 
        % gripper_command 
        gripper_command
        
        % gripper_pressure_lower_limit - lowest 'effort' that should be applied 
        % to the gripper if gripper_pressure is set to 0; it should be high 
        % enough to open/close the gripper (~150 PWM or ~400 mA current)
        gripper_pressure_lower_limit double
        
        % gripper_pressure_upper_limit - largest 'effort' that should be 
        % applied to the gripper if gripper_pressure is set to 1; it should be 
        % low enough that the motor doesn't 'overload' when gripping an object 
        % for a few seconds (~350 PWM or ~900 mA)
        gripper_pressure_upper_limit double
        
        % gripper_value - pressure of the gripper
        gripper_value double
        
        % left_finger_index - the index corresponding to the left finger in the 
        % core.js_index_map
        left_finger_index double
        
        % left_finger_lower_limit - the lower positional limits of the left finger
        left_finger_lower_limit double
        
        % left_finger_upper_limit - the upper positional limits of the left finger
        left_finger_upper_limit double
        
        % gripper_state_timer - a timer that controls the execution of the 
        % gripper_state function
        gripper_state_timer timer
    end
    
    methods
        function obj = InterbotixGripperXSInterface( ...
            core, gripper_name, opts)
        % Constructor for the InterbotixGripperXSInterface object
            arguments
                % core - Reference to the InterbotixRobotXSCore class 
                % containing the internal ROS plumbing that drives the MATLAB API
                core InterbotixRobotXSCore
                
                % gripper_name - name of the gripper joint as defined in the 
                % 'motor_config' yaml file; typically, this is 'gripper'
                gripper_name string
                
                % gripper_pressure - fraction from 0 - 1 where '0' means the 
                % gripper operates at 'gripper_pressure_lower_limit' and '1' 
                % means the gripper operates at 'gripper_pressure_upper_limit'
                opts.gripper_pressure double = 0.5
                
                % gripper_pressure_lower_limit - lowest 'effort' that should be 
                % applied to the gripper if gripper_pressure is set to 0; it 
                % should be high enough to open/close the gripper (~150 PWM or 
                % ~400 mA current)
                opts.gripper_pressure_lower_limit double = 150
                
                % gripper_pressure_upper_limit - largest 'effort' that should 
                % be applied to the gripper if gripper_pressure is set to 1; 
                % it should be low enough that the motor doesn't 'overload' 
                % when gripping an object for a few seconds (~350 PWM or ~900 mA)
                opts.gripper_pressure_upper_limit double = 350
            end
            obj.gripper_name = gripper_name;
            obj.core = core;
            srv = rosmessage("interbotix_xs_msgs/RobotInfoRequest");
            srv.CmdType = "single";
            srv.Name = gripper_name;
            gripper_info = obj.core.srv_get_info.call(srv);

            % Inform the user that they must be using current or pwm mode if not
            if (gripper_info.Mode ~= "current" && gripper_info.Mode ~= "pwm")
                disp("Please set the gripper's 'operating mode' to 'pwm' or 'current'.")
            end
            obj.gripper_moving = false;

            % set up gripper_command message
            obj.gripper_command = rosmessage("interbotix_xs_msgs/JointSingleCommand");
            obj.gripper_command.Name = "gripper";

            % calculate gripper_value (lower + pressure * (upper - lower))
            obj.gripper_pressure_lower_limit = opts.gripper_pressure_lower_limit;
            obj.gripper_pressure_upper_limit = opts.gripper_pressure_upper_limit;
            obj.gripper_value = opts.gripper_pressure_lower_limit + ( ...
                opts.gripper_pressure * ( ...
                    opts.gripper_pressure_upper_limit - opts.gripper_pressure_lower_limit));
            
            % get gripper joint information
            gripper_info_name = gripper_info.JointNames(1);
            obj.left_finger_index = obj.core.js_index_map(gripper_info_name{1});
            obj.left_finger_lower_limit = gripper_info.JointLowerLimits(1);
            obj.left_finger_upper_limit = gripper_info.JointUpperLimits(1);

            obj.set_timer_function();
            fprintf("\nGripper Name: %s\nGripper Pressure: %d%%\n", ...
                gripper_name, opts.gripper_pressure * 100)
            fprintf("Initialized InterbotixGripperXSInterface!\n")
        end

        function set_timer_function(obj)
        % set_timer_function function used to set up the gripper_state_timer
            arguments
                obj InterbotixGripperXSInterface
            end
            obj.gripper_state_timer = timer( ...
                'executionMode','fixedRate', ...
                'Name', 'GripperStateTimer', ...
                'Tag', obj.gripper_name,     ...
                'Period', 0.02,              ...
                'TimerFcn', @obj.gripper_state);
            obj.gripper_state_timer.start();
        end
        
        function gripper_state(obj, ~, ~)
        % gripper_state - Timer Callback function to stop the gripper moving 
        % past its limits when in PWM mode
            arguments
                obj InterbotixGripperXSInterface
                ~
                ~
            end
            
            % check if the gripper is moving
            if (obj.gripper_moving)

                % get gripper current position
                gripper_pos = obj.core.joint_states.Position(obj.left_finger_index);

                % stop the gripper's movement if it hits a limit
                if (((obj.gripper_command.Cmd > 0) && (gripper_pos >= obj.left_finger_upper_limit)) || ...
                    ((obj.gripper_command.Cmd < 0) && (gripper_pos <= obj.left_finger_lower_limit)))
                    obj.gripper_command.Cmd = 0.0;
                    obj.core.pub_single.send(obj.gripper_command);
                    obj.gripper_moving = false;
                end
            end
        end
        
        function gripper_controller(obj, effort, delay)
        % gripper_controller Helper function used to publish effort commands 
        % to the gripper (when in 'pwm' or 'current' mode)
            arguments
                obj InterbotixGripperXSInterface
                
                % effort - effort command to send to the gripper motor
                effort double
                
                % delay - number of seconds to wait before returning control to 
                % the user
                delay double
            end

            obj.gripper_command.Cmd = effort;
            gripper_pos = obj.core.joint_states.Position(obj.left_finger_index);
            
            % checks if the gripper command is compatible with the gripper's 
            %   current position
            if ((obj.gripper_command.Cmd > 0 && gripper_pos < obj.left_finger_upper_limit) || ...
                (obj.gripper_command.Cmd < 0 && gripper_pos > obj.left_finger_lower_limit))
                
                % send the gripper command
                obj.core.pub_single.send(obj.gripper_command);

                % set the gripper_moving bool to true
                obj.gripper_moving = true;

                % pause for the delay
                pause(delay)
            end
        end

        function set_pressure(obj, pressure)
        % set_pressure Set the amount of pressure that the gripper should use 
        % when grasping an object (when in 'effort' control mode)
            arguments
                obj InterbotixGripperXSInterface

                % pressure - a scaling factor from 0 to 1 where the pressure 
                % increases as the factor increases
                pressure double
            end

            % calculates the gripper_value (lower + pressure * (upper - lower))
            obj.gripper_value = obj.gripper_pressure_lower_limit + pressure * ...
                (obj.gripper_pressure_upper_limit - obj.gripper_pressure_lower_limit);
        end

        function open(obj, delay)
        % open Opens the gripper (when in 'pwm' control mode)
            arguments
                obj InterbotixGripperXSInterface

                % delay - number of seconds to delay before returning control 
                % to the user
                delay double = 1.0
            end
            obj.gripper_controller(obj.gripper_value, delay)
        end

        function close(obj, delay)
        % close Closes the gripper (when in 'pwm' control mode)
            arguments
                obj InterbotixGripperXSInterface

                % delay - number of seconds to delay before returning control 
                % to the user
                delay double = 1.0
            end
            obj.gripper_controller(-obj.gripper_value, delay)
        end
    end
end

classdef InterbotixGripperXS < handle
% Standalone Module to control an Interbotix Gripper using PWM or Current control
    properties
        % dxl - reference to the InterbotixRobotXSCore class containing the 
        % internal ROS plumbing that drives the MATLAB API
        dxl InterbotixRobotXSCore
        
        % gripper - Reference to the class's InterbotixGripperXSInterface object
        gripper InterbotixGripperXSInterface
        
        % gripper_name - Name of the gripper joint as defined in the 
        % 'motor_config' yaml file; typically, this is 'gripper'
        gripper_name
    end
    
    methods
        function obj = InterbotixGripperXS( ...
                robot_model, gripper_name, robot_name, gripper_pressure, ...
                gripper_pressure_lower_limit, gripper_pressure_upper_limit, ...
                init_node)
        % Constructor for the InterbotixGripperXS object
            arguments
                % robot_model - Interbotix Arm model (ex. 'wx200' or 'vx300s')
                robot_model string
                
                % gripper_name - name of the gripper joint as defined in the 
                % 'motor_config' yaml file; typically, this is 'gripper'
                gripper_name string
                
                % robot_name - defaults to value given to 'robot_model'; this 
                % can be customized to best suit the user's needs
                robot_name string = ""
                
                % gripper_pressure - fraction from 0 - 1 where '0' means the 
                % gripper operates at 'gripper_pressure_lower_limit' and '1' 
                % means the gripper operates at 'gripper_pressure_upper_limit'
                gripper_pressure double = 0.5
                
                % gripper_pressure_lower_limit - lowest 'effort' that should be 
                % applied to the gripper if gripper_pressure is set to 0; it 
                % should be high enough to open/close the gripper (~150 PWM or 
                % ~400 mA current)
                gripper_pressure_lower_limit double = 150
                
                % gripper_pressure_upper_limit - largest 'effort' that should 
                % be applied to the gripper if gripper_pressure is set to 1; 
                % it should be low enough that the motor doesn't 'overload' 
                % when gripping an object for a few seconds (~350 PWM or ~900 mA)
                gripper_pressure_upper_limit double = 350
                
                % init_node - set to True if the InterbotixRobotXSCore class 
                % should initialize the ROS node; set to False
                init_node {mustBeNumericOrLogical} = 1
            end
            obj.gripper_name = gripper_name;

            % create the core dxl object
            obj.dxl = InterbotixRobotXSCore( ...
                robot_model, robot_name, init_node);
            
            % create the gripper interface object
            obj.gripper = InterbotixGripperXSInterface( ...
                obj.dxl, gripper_name, gripper_pressure, ...
                gripper_pressure_lower_limit, gripper_pressure_upper_limit);
        end

        function success = stop_timers(obj)
            % stop_timers Stops all timers with the gripper_name tag
            % 
            % returns success - indication of whether or not the timers were stopped
            success = true;
            try
                if ~isempty(obj.gripper)
                    gripper_timers = timerfind("Tag", obj.gripper_name);
                    if ~isempty(gripper_timers)
                        stop(gripper_timers)
                        fprintf("%s stopped successfully.\n", gripper_timers(:).Name)
                        delete(gripper_timers);
                        fprintf("All gripper timers deleted successfully.\n")
                    else
                        fprintf("\nNo timers to delete in gripper.\n")
                    end
                    fprintf("All timers stopped and deleted.\n")
                else
                    fprintf("Gripper has no timers to be stopped.\n")
                end
            catch ME
                success = false;
                fprintf("Something went wrong when stopping timers. Run `delete(timerfindall)` to stop and delete all timers instead.\n")
                rethrow(ME)
            end
        end

    end
end

classdef trajectoryChecker
    %{
        Assignment Group 20
        trajectoryChecker class: Checks a trajectory object for constraint violations 
    
        True/false is return for mechanical constraints requiring the user
        to adjust via points
        
        An array of boolean is returned for torque constraints
        
        trajectoryChecker is Static
    %}

    methods(Static)
        % inputs: 
        % robot: robot object that contains constraint properties
        % 
        function [taskCheck jointCheck torqueCheck] = trajectoryChecker(robot, taskTrajectory, jointTrajectory)
            
        end
    end
    
    methods(Access = private)
        function [taskCheck] = taskChecker(taskTrajectory)
            
        end
        
        function [jointCheck] = jointChecker(jointTrajectory)
            
        end
        
        function [torqueCheck] = torqueChecker(jointTrajectory)
            
        end
        
    end
    
end


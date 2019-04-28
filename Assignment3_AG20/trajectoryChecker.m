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
        function [taskCheck, jointCheck, torqueCheck] = trajectoryChecker(robot, taskTrajectory, jointTrajectory)
            taskCheck = obj.taskChecker(taskTrajectory,robot); 
            jointCheck = obj.jointChecker(jointTrajectory,robot);
            torqueCheck = obj.jointChecker(jointTrajectory,robot); 
        end
    end
    
    methods(Access = private)
        function [taskCheck] = taskChecker(taskTrajectory)
            
        end
        
        % Check whether joints exceed limits
        function [jointCheck, n] = jointChecker(obj,jointTrajectory, robot)
            n = [];
            
            q1 = jointTrajectory.getQ1();
            q2 = jointTrajectory.getQ2();
            q3 = jointTrajectory.getQ3();
            q4 = jointTrajectory.getQ4();
            q5 = jointTrajectory.getQ5();
            
            for i = 1:length(getTimeseries())
                [check1] = obj.checkJointLimit(q1(i),robot.j1_lim);
                [check2] = obj.checkJointLimit(q2(i),robot.j2_lim);
                [check3] = obj.checkJointLimit(q3(i),robot.j3_lim);
                [check4] = obj.checkJointLimit(q4(i),robot.j4_lim);
                [check5] = obj.checkJointLimit(q5(i),robot.j5_lim);
                jointCheckN = check1 && check2 && check3 && check4 && check5;
                if ~jointCheckN
                    n = [n i];
                    jointCheck = 0;
                end
            end
        end
        
        % Check whether torque demand is too high
        function [torqueCheck] = torqueChecker(jointTrajectory)
            
        end
        
        
        
    end
    
    methods(Static)
        % Check whether joint angle is inside of limits
        function [check] = checkJointLimit(q,q_lim)
            check = ~(q>max(q_lim) || q<min(q_lim));
        end
    end
    
end


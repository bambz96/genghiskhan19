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
            t = taskTrajectory.getTimeseries(); 
            position = taskTrajectory.getPosition(); 
            x = position(1,:); 
            y = position(2,:);
            z = position(3,:);
            theta = position(4,:);
            
            
            EE_FL = [EE_length/2;
                     EE_width/2;
                     0];
            EE_FR = [EE_length/2;
                     -EE_width/2;
                     0];
            EE_RL = [-EE_length/2;
                     EE_width/2;
                     0];
            EE_RR = [-EE_length/2;
                     -EE_width/2;
                     0];
            
            taskCheck = 1;
            for i = 1:length(t)
                R = [cosd(theta(i)) -sind(theta(i))   0;
                sind(theta(i)) cosd(theta(i))     0;
                0           0               1];
                EE_FL_rot = R*EE_FL;
                EE_FR_rot = R*EE_FR;
                EE_RL_rot = R*EE_RL; 
                EE_RR_rot = R*EE_RR; 
                pointCheck1 = pointCheckerBase(EE_FL_rot(1), EE_FL_rot(2), EE_FL_rot(3));
                pointCheck2 = pointCheckerBase(EE_FR_rot(1), EE_FR_rot(2), EE_FR_rot(3));
                pointCheck3 = pointCheckerBase(EE_RL_rot(1), EE_RL_rot(2), EE_RL_rot(3));
                pointCheck4 = pointCheckerBase(EE_RR_rot(1), EE_RR_rot(2), EE_RR_rot(3));
                taskCheck = taskCheck&&pointCheck1&&pointCheck2&&pointCheck3&&pointCheck4;
            end
        end
        
        function [pointCheck] = pointCheckerBase(robot,x,y,z)
            pointCheckX = (x<robot.baseX-robot.baseLength/2) && (x>robot.baseX+robot.baseLength/2);
            pointCheckY = (y<robot.baseY-robot.baseWidth/2) && (y>robot.baseY+robot.baseWidth/2);
            pointCheckZ = (z>robot.baseHeight);
            pointCheck = pointCheckX||pointCheckY||pointCheckZ;
        end
        
        % Check whether joints exceed limits
        function [jointCheck, n] = jointChecker(obj,jointTrajectory, robot)
            n = [];
            
            Q = jointTrajectory.getQ(); 
            
            q1 = Q(1,:);
            q2 = Q(2,:);
            q3 = Q(3,:);
            q4 = Q(4,:);
            q5 = Q(5,:);
            
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
        function [torqueCheck] = torqueChecker(jointTrajectory, taskTrajectory)
            convertToDegS = 10000*180/pi; 
            Q_ddot = jointTrajectory.getQ_ddot;
            Q = jointTrajectory.getQ(); 
            Q_ddot1 = Q_ddot(1,:);
            Q_ddot2 = Q_ddot(2,:);
            Q_ddot3 = Q_ddot(3,:);
            Q_ddot4 = Q_ddot(4,:);
            Q_ddot5 = Q_ddot(5,:);
            
            torqueCheck = 1; 
            position = taskTrajectory.getPosition;
            x = position(1);
            y = position(2);
            z = position(3);
            theta = position(4);
            
            for i = 1:length(Q_ddot)
                staticTorque = staticTorque(x(i),y(i),z(i),theta(i));
                T1 = robot.XL430_T_Max-staticTorque.getT1;
                q1_ddot_Check = convertToDegS*T1/staticTorque.getI1>Q_ddot1(i);
                T2 = robot.XL430_T_Max-staticTorque.getT2;
                q2_ddot_Check = convertToDegS*T2/staticTorque.getI2>Q_ddot2(i);
                T3 = robot.XL430_T_Max-staticTorque.getT3;
                q3_ddot_Check = convertToDegS*T3/staticTorque.getI3>Q_ddot3(i);
                T4 = robot.XL430_T_Max-staticTorque.getT4;
                q4_ddot_Check = convertToDegS*T4/staticTorque.getI4>Q_ddot4(i);
                T5 = robot.XL430_T_Max-staticTorque.getT5;
                q5_ddot_Check = convertToDegS*T5/staticTorque.getI5>Q_ddot5(i);
                torqueCheck = torqueCheck&&q1_ddot_Check&&q2_ddot_Check&&q3_ddot_Check&&q4_ddot_Check&&q5_ddot_Check;
            end
                
        end
        
        
        
    end
    
    methods(Static)
        % Check whether joint angle is inside of limits
        function [check] = checkJointLimit(q,q_lim)
            check = ~(q>max(q_lim) || q<min(q_lim));
        end
    end
    
end


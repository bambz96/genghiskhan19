classdef staticTorque < handle
    
    %                   <-L_PL-> <---L3---> {m_4}
    %       {m_PL_top} o--------o---------o   ^
    %           ^     /        / {m_3}    |   |
    %           |    /        /           |   L4
    %           L2  /        /            -   |
    %           |  /        /            | |  v
    %           v /        /       {End Effector}
    % {m_PL_bot} o--------o 
    %                     |
    %                   /////
    
    properties 
        robot
        m_5    % Mass of end effector + last joint
        
    end
    
    methods
        
        function obj = staticTorque(robot)
            obj.robot = robot;
            obj.m_5 = robot.m_4+robot.m_E;
        end
        
        function [T_2,M_PL_bot] = findStaticMotorTorque(obj,x,y,z,theta_in)
            g = obj.robot.g; 
            Nmm_to_Nm = 1/1000;
            [q1_calc, q2_calc, q3_calc, q4_calc, q5_calc] = obj.robot.inverseKinematics.findQ(x,y,z,theta_in);
            
            [x_calc,y_calc,z_calc] = obj.robot.forwardKinematics.findCoordinates(q1_calc,q2_calc,q3_calc,q4_calc,q5_calc);
            
            x_34 = x_calc(4) - x_calc(3); % X vector between joint 3 and 4
            y_34 = y_calc(4) - y_calc(3); % Y vector between joint 3 and 4
            z_34 = z_calc(4) - z_calc(3); % Z vector between joint 3 and 4
            
            mag = sqrt(x_34^2+y_34^2+z_34^2); % Magnitude of vector between j3 and j4
            
            P_34_u = [x_34/mag y_34/mag z_34/mag]; % Unit vector 
            P_PL_top = [x_calc(3) y_calc(3) z_calc(3)] - obj.robot.L_PL*P_34_u; % Vector to upper parallelogram joint
            P_PL_bot = [x_calc(1) y_calc(1) z_calc(1)] - obj.robot.L_PL*P_34_u; % Vector to lower parallelogram joint
            
            r_PL_top = sqrt(P_PL_top(1)^2+P_PL_top(2)^2);
            r_PL_bot = sqrt(P_PL_bot(1)^2+P_PL_bot(2)^2);
            r_3 = sqrt(x_calc(3)^2+y_calc(3)^2);
            r_E = sqrt(x_calc(4)^2+y_calc(4)^2);
            
            % Moments generated around q2  
            M_PL_top = sign(P_PL_top(1))*g*r_PL_top*obj.robot.m_PL_top;
            M_PL_bot = sign(P_PL_bot(1))*g*r_PL_bot*obj.robot.m_PL_bot;
            M_j3 = g*r_3*obj.robot.m_3;
            M_E = g*r_E*obj.m_5;
            T_tSpring = obj.robot.tSpring_K*(-q2_calc-obj.robot.tSpring_0);
            
            % Sum of moments
            T_2 = -(M_PL_top+M_PL_bot+M_j3+M_E-T_tSpring)*Nmm_to_Nm; 
            M_PL_bot = -(M_PL_top+M_j3+M_E)*Nmm_to_Nm;
        end
    end
end
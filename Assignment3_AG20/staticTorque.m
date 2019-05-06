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
    
    properties(Access = private) 
        m_5    % Mass of end effector + last joint
        T1
        T2
        T3
        T4
        T5
        I1
        I2
        I3
        I4
        I5
    end
    
    properties(Constant)
        g = 9.81;
        Nmm_to_Nm = 1/1000; 
    end
    
    methods
        function obj = staticTorque(robot,x,y,z,theta_in)
            obj.m_5 = robot.m_4+robot.m_E;
            
            [q1_calc, q2_calc, q3_calc, q4_calc, q5_calc] = robot.inverseKinematics.findQ(x,y,z,theta_in);
            [x_calc,y_calc,z_calc] = robot.forwardKinematics.findCoordinates(q1_calc,q2_calc,q3_calc,q4_calc,q5_calc);
            
            x_34 = x_calc(4) - x_calc(3); % X vector between joint 3 and 4
            y_34 = y_calc(4) - y_calc(3); % Y vector between joint 3 and 4
            z_34 = z_calc(4) - z_calc(3); % Z vector between joint 3 and 4
            
            mag = sqrt(x_34^2+y_34^2+z_34^2); % Magnitude of vector between j3 and j4
            
            P_34_u = [x_34/mag y_34/mag z_34/mag]; % Unit vector 
            P_PL_top = [x_calc(3) y_calc(3) z_calc(3)] - robot.L_PL*P_34_u; % Vector to upper parallelogram joint
            P_PL_bot = [x_calc(1) y_calc(1) z_calc(1)] - robot.L_PL*P_34_u; % Vector to lower parallelogram joint
            
            r_PL_top = sqrt(P_PL_top(1)^2+P_PL_top(2)^2);
            r_PL_bot = sqrt(P_PL_bot(1)^2+P_PL_bot(2)^2);
            r_3 = sqrt(x_calc(3)^2+y_calc(3)^2);
            r_E = sqrt(x_calc(4)^2+y_calc(4)^2);
            
            % Moments generated around q2  
            M_PL_top = sign(P_PL_top(1))*obj.g*r_PL_top*robot.m_PL_top;
            M_PL_bot = sign(P_PL_bot(1))*obj.g*r_PL_bot*robot.m_PL_bot;
            M_j3 = obj.g*r_3*robot.m_3;
            M_E = obj.g*r_E*obj.m_5;
            T_tSpring = robot.tSpring_K*(-q2_calc-robot.tSpring_0);
            
            % Sum of moments for T2
            obj.T2 = (M_PL_top+M_PL_bot+M_j3+M_E-T_tSpring)*obj.Nmm_to_Nm;
            
            % Moments generated around q3
            M_E = obj.g*(r_E-r_3)*obj.m_5;
            M_PL_top = -(sign(P_PL_top(1)))*(-r_3+r_PL_top)*obj.g*robot.m_PL_top;
            M_PL_bot = -(sign(P_PL_top(1)))*(-r_3+r_PL_bot)*obj.g*robot.m_PL_bot;
            
            % Sum of moments for T3
            obj.T3 = (M_E-M_PL_top-M_PL_bot)*obj.Nmm_to_Nm;
            
            % Other Moments
            obj.T1 = 0;
            obj.T4 = 0;
            obj.T5 = 0;
            
            % Get Moments of inertia around each joint
            
            % I1 
            obj.I1 = obj.m_5*r_E^2+robot.m_3*r_3^2+robot.m_PL_bot*r_PL_bot^2+robot.m_PL_top*r_PL_bot^2;
            
            
            %I2
            dist_q2_PL_top = sqrt(r_PL_top^2+(P_PL_top(3)-robot.L1)^2);
            dist_q2_PL_bot = sqrt(r_PL_bot^2+(P_PL_bot(3)-robot.L1)^2);
            dist_q2_EE = sqrt(r_E^2+(P_PL_bot(3)-robot.L1)^2);
            I_2_PL_Bot = robot.m_PL_bot*robot.L_PL^2;
            I_2_PL_Top = robot.m_PL_top*dist_q2_PL_top^2;
            I_2_E = obj.m_5*dist_q2_EE^2;
            I_2_3 = robot.m_3*robot.L2^2;
            obj.I2 = I_2_PL_Bot+I_2_PL_Top+I_2_E+I_2_3;
            
            
            %I3
            obj.I3 = robot.m_PL_bot*robot.L_PL^2+robot.m_PL_top*robot.L_PL^2+obj.m_5*robot.L3^2;
            
            %I4
            obj.I4 = obj.m_5*robot.L4*2; 
            
            %I5
            obj.I5 = 1.648E5/1000; 
        end
        
        function T1 = getT1(obj)
            T1 = obj.T1;
        end
        
        function T2 = getT2(obj)
            T2 = obj.T2;
        end
        
        function T3 = getT3(obj)
            T3 = obj.T3;
        end
        
        function T4 = getT4(obj)
            T4 = obj.T4;
        end
        
        function T5 = getT5(obj)
            T5 = obj.T5;
        end
        
        function I1 = getI1(obj)
            I1 = obj.I1; 
        end
        
        function I2 = getI2(obj)
            I2 = obj.I2; 
        end
        
        function I3 = getI3(obj)
            I3 = obj.I3; 
        end
        
        function I4 = getI4(obj)
            I4 = obj.I4; 
        end
        
        function I5 = getI5(obj)
            I5 = obj.I5; 
        end
        
%         function [staticTorque] = findStaticMotorTorque(obj,x,y,z,theta_in)
%             g = obj.robot.g; 
%             Nmm_to_Nm = 1/1000;
%             [q1_calc, q2_calc, q3_calc, q4_calc, q5_calc] = obj.robot.inverseKinematics.findQ(x,y,z,theta_in);
%             
%             [x_calc,y_calc,z_calc] = obj.robot.forwardKinematics.findCoordinates(q1_calc,q2_calc,q3_calc,q4_calc,q5_calc);
%             
%             x_34 = x_calc(4) - x_calc(3); % X vector between joint 3 and 4
%             y_34 = y_calc(4) - y_calc(3); % Y vector between joint 3 and 4
%             z_34 = z_calc(4) - z_calc(3); % Z vector between joint 3 and 4
%             
%             mag = sqrt(x_34^2+y_34^2+z_34^2); % Magnitude of vector between j3 and j4
%             
%             P_34_u = [x_34/mag y_34/mag z_34/mag]; % Unit vector 
%             P_PL_top = [x_calc(3) y_calc(3) z_calc(3)] - obj.robot.L_PL*P_34_u; % Vector to upper parallelogram joint
%             P_PL_bot = [x_calc(1) y_calc(1) z_calc(1)] - obj.robot.L_PL*P_34_u; % Vector to lower parallelogram joint
%             
%             r_PL_top = sqrt(P_PL_top(1)^2+P_PL_top(2)^2);
%             r_PL_bot = sqrt(P_PL_bot(1)^2+P_PL_bot(2)^2);
%             r_3 = sqrt(x_calc(3)^2+y_calc(3)^2);
%             r_E = sqrt(x_calc(4)^2+y_calc(4)^2);
%             
%             % Moments generated around q2  
%             M_PL_top = sign(P_PL_top(1))*g*r_PL_top*obj.robot.m_PL_top;
%             M_PL_bot = sign(P_PL_bot(1))*g*r_PL_bot*obj.robot.m_PL_bot;
%             M_j3 = g*r_3*obj.robot.m_3;
%             M_E = g*r_E*obj.m_5;
%             T_tSpring = obj.robot.tSpring_K*(-q2_calc-obj.robot.tSpring_0);
%             
%             % Sum of moments
%             T_2 = -(M_PL_top+M_PL_bot+M_j3+M_E-T_tSpring)*Nmm_to_Nm; 
%             M_PL_bot = -(M_PL_top+M_j3+M_E)*Nmm_to_Nm;
%         end
    end
end
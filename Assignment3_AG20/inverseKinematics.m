classdef inverseKinematics <handle
    %INVERSEKINEMATICS Calculates Inverse Kinematics Stuff
    %   Detailed explanation goes here
    
    properties
        T_0E
        
        q1      = sym('q1'); % deg
        q2      = sym('q2'); % deg  
        q3      = sym('q3'); % deg  
        q4      = sym('q4'); % deg  
        q5      = sym('q5'); % deg  
        
        q1_sol
        q2_sol
        q3_sol
        q4_sol
        q5_sol
        
        q1_f
        q2_f
        q3_f
        q4_f
        q5_f
        
        x       = sym('x'); % mm
        y       = sym('y'); % mm
        z       = sym('z'); % mm
        theta   = sym('theta'); % deg
        
    end
    
    methods
        function obj = inverseKinematics(T_0E, robot)
           obj.T_0E = T_0E;
           [obj.q1_sol, obj.q2_sol, obj.q3_sol, obj.q4_sol, obj.q5_sol] = symFindQ(obj,robot);
           obj.q1_f = matlabFunction(obj.q1_sol);
           obj.q2_f = matlabFunction(obj.q2_sol);
           obj.q3_f = matlabFunction(obj.q3_sol);
           obj.q4_f = matlabFunction(obj.q4_sol);
           obj.q5_f = matlabFunction(obj.q5_sol);
        end
        
        function [q1_sol, q2_sol, q3_sol, q4_sol, q5_sol] = symFindQ(obj, robot)
           radToDeg = 180/pi;
           %% q1
           q1_sol = atan2(obj.y,obj.x);
           q1_sol = radToDeg*q1_sol; 
           
           %% q3
           L_b = sqrt(obj.x^2+obj.y^2+(obj.z+robot.L4-robot.L1)^2);
           
           % Cosine rule
           beta = obj.cosineRule(L_b,robot.L2,robot.L3);
           alpha = obj.cosineRule(robot.L3,L_b,robot.L2);
           %gamma = obj.cosineRule(robot.L2,L_b,robot.L3);

           q3_sol = beta - 90; 
           
           %% q2
           P_AW_z = obj.z+robot.L4-robot.L1;
           P_AW_xy = sqrt(obj.x^2+obj.y^2);
           theta_24 = atan2(P_AW_z,P_AW_xy);
           theta_24 = radToDeg*theta_24;
           q2_sol = -(90-theta_24-alpha);
          
           %% q4
           q4_sol = -(q2_sol+q3_sol);
           
           %% q5
           q5_sol = obj.theta-q1_sol; 
        end
        
%         function [q1, q2, q3, q4, q5] = findQ(obj, x,y,z,theta)
%             q1 = double(subs(obj.q1_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
%             q2 = double(subs(obj.q2_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
%             q3 = double(subs(obj.q3_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
%             q4 = double(subs(obj.q4_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
%             q5 = double(subs(obj.q5_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
%         end
        
        % Solve for joint angles based on robot pose (Note: using
        % matlabFunction is faster than subs)
        function [q1, q2, q3, q4, q5] = findQ(obj, x,y,z,theta)
            q1 = obj.q1_f(x,y);
            q2 = obj.q2_f(x,y,z);
            q3 = obj.q3_f(x,y,z);
            q4 = obj.q4_f(x,y,z);
            q5 = obj.q5_f(theta,x,y);
        end
        
        function T_2 = findQ2torque(obj,robot,x,y,z,theta_in)
            L_PL = 100;
            g = 9.81;
            m_PL_top = 0.075;
            m_counter = 0.1;
            m_PL_bot = 0.085+m_counter;
            m_3 = 0.054;
            m_EE_reduct = 0;%0.04+0.02+0.008+0.005;
            m_4 = 0.063+0.206-m_EE_reduct;
            
            K_torsionSpring = 12; % Nmm/deg;
            Nmm_to_Nm = 1/1000;
            torsionSpringOffset = 0;
            
            [q1_calc, q2_calc, q3_calc, q4_calc, q5_calc] = findQ(obj,x,y,z,theta_in);
            [x_calc,y_calc,z_calc] = robot.forwardKinematics.findCoordinates(q1_calc,q2_calc,q3_calc,q4_calc,q5_calc);
            
            deltaX = x_calc(4) - x_calc(3);
            deltaY = y_calc(4) - y_calc(3);
            deltaZ = z_calc(4) - z_calc(3);
            
            mag = sqrt(deltaX^2+deltaY^2+deltaZ^2);
            
            P_34_u = [deltaX/mag deltaY/mag deltaZ/mag];
            P_PL_top = [x_calc(3) y_calc(3) z_calc(3)] - L_PL*P_34_u;
            P_PL_bot = [x_calc(1) y_calc(1) z_calc(1)] - L_PL*P_34_u;
            
            T_PL_top = sign(P_PL_top(1))*g*sqrt(P_PL_top(1)^2+P_PL_top(2)^2)*m_PL_top;
            T_PL_bot = sign(P_PL_bot(1))*g*sqrt(P_PL_bot(1)^2+P_PL_bot(2)^2)*m_PL_bot;
            T_3 = g*sqrt(x_calc(3)^2+y_calc(3)^2)*m_3;
            T_E = g*sqrt(x_calc(4)^2+y_calc(4)^2)*m_4;
            
            T_torsion = (K_torsionSpring)*(-q2_calc-torsionSpringOffset);
            
            T_2 = -(T_PL_top+T_PL_bot+T_3+T_E-T_torsion)*Nmm_to_Nm; 
        end
        
    end
    
    methods(Static) 
        function angle = cosineRule(L1,L2,L3)
            % L1 is opposite required angle
            radToDeg = 180/pi;
            angle = acos((L2^2+L3^2-L1^2)/(2*L2*L3));
            angle = angle*radToDeg;
        end
        
    end
       
end


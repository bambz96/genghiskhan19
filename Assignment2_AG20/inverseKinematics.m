classdef inverseKinematics
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
        
        x       = sym('x'); % mm
        y       = sym('y'); % mm
        z       = sym('z'); % mm
        theta   = sym('theta'); % deg
        
    end
    
    methods
        function obj = inverseKinematics(T_0E, robot)
           obj.T_0E = T_0E;
           [obj.q1_sol, obj.q2_sol, obj.q3_sol, obj.q4_sol, obj.q5_sol] = symFindQ(obj,robot);
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
        
        function [q1, q2, q3, q4, q5] = findQ(obj, x,y,z,theta)
            q1 = double(subs(obj.q1_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
            q2 = double(subs(obj.q2_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
            q3 = double(subs(obj.q3_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
            q4 = double(subs(obj.q4_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
            q5 = double(subs(obj.q5_sol,[obj.x,obj.y,obj.z,obj.theta],[x,y,z,theta]));
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


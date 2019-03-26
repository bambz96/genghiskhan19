classdef inverseKinematics
    %INVERSEKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        T_0E
        
        q1      = sym('q1'); % deg
        q2      = sym('q2'); % deg  
        q3      = sym('q3'); % deg  
        q4      = sym('q4'); % deg  
        q5      = sym('q5'); % deg  
        
        x       = sym('x'); % mm
        y       = sym('y'); % mm
        z       = sym('z'); % mm
        theta   = sym('theta'); % deg
        
    end
    
    methods
        function obj = inverseKinematics(T_0E)
           obj.T_0E = T_0E;
        end
        
        function [q1_sol, q2_sol, q3_sol, q4_sol, q5_sol] = symSolveQ(obj)
           degToRad = pi/180;
           T_0E_cmnd = [cos(obj.theta*degToRad) -sin(obj.theta*degToRad) 0  obj.x;
                        sin(obj.theta*degToRad) cos(obj.theta*degToRad)  0  obj.y;
                        0                   0                            1  obj.z;
                        0                   0                            0  1];
                    
           eqns = sym('a',[1,16]);
           vars = [obj.q1, obj.q2, obj.q3, obj.q4, obj.q5];
           counter = 0;
           
           for i = 1:4
               for j = 1:4
                   counter = counter + 1; 
                   newEqn = (obj.T_0E(i,j)==T_0E_cmnd(i,j));
%                    newEqn = rewrite
                    eqns(counter) = newEqn;
               end
           end
           
           [q1_sol, q2_sol, q3_sol, q4_sol, q5_sol] = solve(eqns, vars,'IgnoreAnalyticConstraints', true);
           
                    
        end
    end
    
end


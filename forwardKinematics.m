classdef forwardKinematics
    % Formulates Transformation matrices, and calculates robot pose based on joint angles
    
    properties
        %% Calculated
        DH                          % DH Table (Matrix)
        
        %% Transformation Matrices
        T_01
        T_12
        T_23
        T_34
        T_45
        T_5E
        
        T_02
        T_03
        T_04
        T_05
        T_0E
        
        %% Vectors
        r01_0 
        r12_1 
        r23_2 
        r34_3 
        r45_4
        r5E_5

        %% Symbolic Joint States
        q1      = sym('q1'); % deg
        q2      = sym('q2'); % deg  
        q3      = sym('q3'); % deg  
        q4      = sym('q4'); % deg  
        q5      = sym('q5'); % deg  
    end
    
    methods
        function obj = forwardKinematics(robot)
            % Generate DH Table
            obj.DH = generateDH(obj,robot);
            
            % Calculate transforms
            obj.T_01 = obj.calculateTransform(obj.DH(1,:));
            obj.T_12 = obj.calculateTransform(obj.DH(2,:));
            obj.T_23 = obj.calculateTransform(obj.DH(3,:));
            obj.T_34 = obj.calculateTransform(obj.DH(4,:));
            obj.T_45 = obj.calculateTransform(obj.DH(5,:));
            obj.T_5E = obj.calculateTransform(obj.DH(6,:));
            
            obj.T_02 = obj.T_01*obj.T_12;
            obj.T_03 = obj.T_02*obj.T_23;
            obj.T_04 = obj.T_03*obj.T_34;
            obj.T_05 = obj.T_04*obj.T_45;
            obj.T_0E = obj.T_05*obj.T_5E;
            
            % Simplify expressions
%             obj.T_01 = simplify(obj.T_01);
%             obj.T_02 = simplify(obj.T_02);
%             obj.T_03 = simplify(obj.T_03);
%             obj.T_04 = simplify(obj.T_04);
%             obj.T_05 = simplify(obj.T_05);
%             obj.T_0E = simplify(obj.T_0E);
%             
            obj.r01_0 = [0;0;robot.L1];
            obj.r12_1 = [0;0;0];
            obj.r23_2 = [robot.L2;0;0];
            obj.r34_3 = [robot.L3;0;0];
            obj.r45_4 = [0;-robot.L4;0];
            obj.r5E_5 = [0;0;robot.LE];
        end
        
        % Use to initialise DH and modify if necessary
        function DH = generateDH(obj,robot)
          DH = [0           0           robot.L1    (obj.q1+robot.q1_O);
                0           90          0           (obj.q2+robot.q2_O);
                robot.L2    0           0           (obj.q3+robot.q3_O);
                robot.L3    0           0           (obj.q4+robot.q4_O);
                0           90          robot.L4    (obj.q5+robot.q5_O);
                0           0           robot.LE    0];
        end
        
        function [x,y,z] = findCoordinates(obj,q1,q2,q3,q4,q5)
            x = zeros(6,1);
            y = zeros(6,1);
            z = zeros(6,1);
            
            coord1 = [obj.r01_0];
            coord2 = obj.T_01*[obj.r12_1;1];
            coord3 = obj.T_02*[obj.r23_2;1];
            coord4 = obj.T_03*[obj.r34_3;1];
            coord5 = obj.T_04*[obj.r45_4;1];
            coordE = obj.T_05*[obj.r5E_5;1];
            
            coord1 = subs(coord1,[obj.q1,obj.q2,obj.q3,obj.q4,obj.q5],[q1,q2,q3,q4,q5]);
            coord2 = subs(coord2,[obj.q1,obj.q2,obj.q3,obj.q4,obj.q5],[q1,q2,q3,q4,q5]);
            coord3 = subs(coord3,[obj.q1,obj.q2,obj.q3,obj.q4,obj.q5],[q1,q2,q3,q4,q5]);
            coord4 = subs(coord4,[obj.q1,obj.q2,obj.q3,obj.q4,obj.q5],[q1,q2,q3,q4,q5]);
            coord5 = subs(coord5,[obj.q1,obj.q2,obj.q3,obj.q4,obj.q5],[q1,q2,q3,q4,q5]);
            coordE = subs(coordE,[obj.q1,obj.q2,obj.q3,obj.q4,obj.q5],[q1,q2,q3,q4,q5]);
            
            x(1) = coord1(1);
            y(1) = coord1(2);
            z(1) = coord1(3); 
            
            x(2) = coord2(1);
            y(2) = coord2(2);
            z(2) = coord2(3); 
            
            x(3) = coord3(1);
            y(3) = coord3(2);
            z(3) = coord3(3); 
            
            x(4) = coord4(1);
            y(4) = coord4(2);
            z(4) = coord4(3); 
            
            x(5) = coord5(1);
            y(5) = coord5(2);
            z(5) = coord5(3); 
            
            x(6) = coordE(1);
            y(6) = coordE(2);
            z(6) = coordE(3); 
            
        end
    end
    
    methods(Static)
        
        function T = calculateTransform(DHrow)
            degToRad = pi/180;
            a = DHrow(1);
            alpha = degToRad*DHrow(2);
            d = DHrow(3);
            q = degToRad*DHrow(4);
            
            Dx =    [1          0           0           a;
                    0           1           0           0;
                    0           0           1           0;
                    0           0           0           1];
                
            Rx =    [1          0           0           0;
                    0           cos(alpha) -sin(alpha)  0;
                    0           sin(alpha) cos(alpha)   0;
                    0           0           0           1];
                
            Dz =    [1          0           0           0;
                    0           1           0           0;
                    0           0           1           d;
                    0           0           0           1];
                
            Rz =    [cos(q)     -sin(q)         0       0;
                     sin(q)      cos(q)         0       0;
                     0              0           1       0;
                     0              0           0       1];
            
            T = Dx*Rx*Dz*Rz;
            T = simplify(T);
        end
        

    end
    
end


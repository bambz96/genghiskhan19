classdef return_trj < robot_trj
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Very basic trajecory implementation
    Defines a trajectory from the previoius block drop location
    Back to the loading bay
    
    At thi spoint this trajectory is pretty much exactly the same as the
    block movement but in reverse
    Should be reworked a little...
    
    NOTE: All coordinates in column vectors
    
    Constructor inputs:
    loadBay:    loading Bay coordinates (5DOF)
    t0:         start time of trajectory
    ts:         sampling time
    length:     total trajectory time 
    block:      jblock that has just been delivered
    
    
    % Things to improve:
        - iterate on parameterized "fractional time approach" 
        - add optimisation for time of major trajectory segments?
        - path optimisation, possibly via gradient descent?
  
    %}
    properties(Constant)
        LiftHeight =    0.020;  % m height of via above loading bay 
        
        % fractional time to "Load" the new block (vf from v3)
        LoadTime =  0.1;        
        % time to clear the block
        ClearBTime =  0.1;      
        % time to "withdraw from" the tower
        WithdrawTime = 0.25  
        %fractional time to reach Path Via
        PathVTime = 0.45;
        
        loadY = -0.025;         % offset for loading position in bay frame    
        loadZ = 0.025;          % offset for loading position in bay frame
        
        MinPathRadius = 0.22;    % minimum Radius for path via

    end 
    properties(Access = private)
        block     % defines the block that has just been placed in the tower
    end
    
    methods 
        function obj = return_trj(loadBay, t0, length, block)
            
            % times at all locations
            t = return_trj.calculateTime(t0, length);
            
            % input all x locations
            x = return_trj.calculatePosition(loadBay, block);
            
            obj = obj@robot_trj(x, t);
            obj.block = block;
        end
   
    end
    
    methods(Static, Access = private)
        
        % Parameterized determination of via point times
        function t = calculateTime(t0, T)
            tf = t0 + T;
            
            %parameter based times
            t1 = t0 + return_trj.ClearBTime*T;
%             t2 = t0 + return_trj.WithdrawTime*T;
            
            t3 = t0 + return_trj.PathVTime*T;

            t4 = tf - return_trj.LoadTime*T;
            
                  % midpoint
            t = [t0, t1, t3, t4, tf];
        end
        
        % Determination of via locations
        function x = calculatePosition(loadingBay, block)
            dropLocation = [block.dropLocation; robot_trj.ClearGrip];
            
            v1 = dropLocation + [0; 0; jBlock.Height; 0; 0];
            
            % v2 = return_trj.withdrawPosition(block);
            
            v4 = return_trj.loadingPosition(loadingBay);
            
            
                        
            v0 = [loadingBay(1:4); robot_trj.OpenGrip];
            v3 = return_trj.pathVia(v4, v1);
            % place all position vectors into an array 
            x = [dropLocation, v1, v3, v4, v0];
        end
        
        
        % Calculates a path via (v3) from two endpoint vias v1 and v3
        % Essentially calulates the via as an average in cylindrical
        % coordinates...
        function Vp = pathVia(v1, v2)
            % calculate radii
            rad1 = norm(v1(1:2));
            radius = norm(v2(1:2));
            radius = (rad1 + radius)/2;
            
            % ensure path via radius is greater than minimum
            if radius < moveBlock_trj.MinPathRadius
                radius = moveBlock_trj.MinPathRadius;
            end
            
            % calculate angles
            theta1 = atan2(v1(2), v1(1));
            theta3 = atan2(v2(2), v2(1));
            theta2 = (theta1 + theta3)/2;
            
            % initialize v2
            Vp = zeros(5,1);
            % calculate x and y
            Vp(1) = radius*cos(theta2);
            Vp(2) = radius*sin(theta2);
            
            % z position is the same as v2
            Vp(3) = v2(3);
            
            % average other values
            Vp(4:5) = (v1(4:5) + v2(4:5))/2;
         
        end
        
        function xL = loadingPosition(loadingBay)
            ZR = return_trj.zRotation(loadingBay(4));
            offset = ZR*[0; return_trj.loadY; return_trj.loadZ];
            xL = loadingBay + [offset; 0; robot_trj.OpenGrip];
        end
        
        % Augments the block approach position with a gripper position
        function xW = withdrawPosition(block)
            withdrawP = block.approachPosition;

            xW = [withdrawP; robot_trj.ClearGrip];
        end
        
    end
    
    methods (Static, Access=private)
        % Just a z rotation, nothing to see here folks
        % takes input in radians
        function R = zRotation(theta)
            R = [cos(theta) -sin(theta) 0;
                 sin(theta) cos(theta) 0;
                 0           0         1];
        end
    end
    
end
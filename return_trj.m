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
        - potentially change the approach/withdrawal strategies
        - integrate with the return, grip, drop, and pause commands
        for a continuous trajectory for an entire pick and place. 
        - add optimisation for time of major trajectory segmentss
        - path optimisation, possibly via gradient descent
        - assess the necessity of via 2 => yes. V Necessary    
    %}
    properties(Constant)
        LiftHeight =    0.020;  % m height of via above loading bay 
        LoadTime =  0.5;        % time to "Approach" the new block (vf from v3)
        WithdrawTime =  0.5;    % time to "withdraw from" the tower (v1 from drop location)
        
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
            t = return_trj.simpleTime(t0, length);
            
            % input all x locations
            x = return_trj.simplePosition(loadBay, block);
            
            obj = obj@robot_trj(x, t);
            obj.block = block;
        end
   
    end
    
    methods(Static, Access = private)
        
        % Simple determination of trajectory times
        function t = simpleTime(t0, length)
            t1 = t0 + return_trj.WithdrawTime;
            tf = t0 + length;
            t3 = tf - return_trj.LoadTime;
            t2 = (t1 + t3)/2;        % midpoint
            t = [t0, t1, t2, t3, tf];
        end
        
        % Simple determination of via locations
        function x = simplePosition(loadingBay, block)
            dropLocation = [block.dropLocation; robot_trj.OpenGrip];
                
            v1 = return_trj.withdrawPosition(block);
            
            v3 = return_trj.loadingPosition(loadingBay);
            
                        
            v0 = [loadingBay(1:4); robot_trj.OpenGrip];
            v2 = return_trj.pathVia(v1, v3);
            % place all position vectors into an array 
            x = [dropLocation, v1, v2, v3, v0];
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
            
            % average other values
            Vp(3:5) = (v1(3:5) + v2(3:5))/2;
         
        end
        
        function xL = loadingPosition(loadingBay)
            ZR = return_trj.zRotation(loadingBay(4));
            offset = ZR*[0; return_trj.loadY; return_trj.loadZ];
            xL = loadingBay + [offset; 0; robot_trj.OpenGrip];
        end
        
        % Augments the block approach position with a gripper position
        function xW = withdrawPosition(block)
            withdrawP = block.approachPosition;

            xW = [withdrawP; robot_trj.OpenGrip];
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
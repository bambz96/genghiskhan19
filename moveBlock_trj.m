classdef moveBlock_trj < robot_trj
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Very basic trajecory implementation
    Defines a trajectory from the Loading bay (taken as input)
    Delivers the block to the drop location determined from the block.
    
    NOTE: All coordinates in column vectors
    
    Constructor inputs:
    loadBay:    loading Bay coordinates (5DOF)
    t0:         start time of trajectory
    ts:         sampling time
    length:     total trajectory time 
    block:      jblock to be delivered
    
    Note: change class to output in meters and radians
    
    
    % Things to improve:
        - potentially change the approach strategy: using a shorter
        approach for second and third blocks, (with an appropriately
        shorter time. 
        - integrate with the return, grip, drop, and pause commands
        for a continuous trajectory for an entire pick and place. 
        - add optimisation for time of major trajectory segmentss
        - path optimisation, possibly via gradient descent
        - assess the necessity of via 2 => yes. V Necessary
    
    %}
    properties(Constant)

        LiftHeight =    0.02;   % m height of via above loading bay
        LiftTime =      0.5;    % time to "pick up" the block (v1 from LB)
        ApproachTime =  0.5;    % time to "approach" the tower (v3 from v2)
        
        MinPathRadius = 0.22;    % minimum Radius for path via
        
        
    end 
    properties(Access = private)
        block           % defines the block to be placed in the tower 
    end
    
    methods 
        function obj = moveBlock_trj(loadBay, t0, length, block)
            
            % times at all locations
            t = moveBlock_trj.simpleTime(t0, length);
            
            % input all x locations
            x = moveBlock_trj.simplePosition(loadBay, block);
            
            obj = obj@robot_trj(x, t);
            obj.block = block;
        end
   
    end
    
    methods(Static, Access = private)
        
        % Simple determination of trajectory times
        function t = simpleTime(t0, length)
            t1 = t0 + moveBlock_trj.LiftTime;
            tf = t0 + length;
            t3 = tf - moveBlock_trj.ApproachTime;
            t2 = (t1 + t3)/2;        % midpoint
            t = [t0, t1, t2, t3, tf];
        end
        
        % Simple determination of via locations
        function x = simplePosition(loadBay, block)
            v0 = [loadBay(1:4); robot_trj.ClosedGrip];
            
            v1 = loadBay + ...
                [0; 0; moveBlock_trj.LiftHeight; 0; robot_trj.ClosedGrip];
            
            dropLocation = [block.dropLocation; robot_trj.ClosedGrip];
            
            v3 = moveBlock_trj.approachPosition(block);
            
            % Now using drop location and pickup average
            v2 = moveBlock_trj.pathVia(v1, v3);
            
            % place all position vectors into an array 
            x = [v0, v1, v2, v3, dropLocation];
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
        
        
        
        % Augments the block approach position with a gripper position
        function xApp = approachPosition(block)
            Approach = block.approachPosition;

            xApp = [Approach; robot_trj.ClosedGrip];
        end
        
    end
    
end
    
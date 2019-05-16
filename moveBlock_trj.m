classdef moveBlock_trj < taskTrajectory
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
        DOF = 5;
        DropHeight =    5;      % mm drop for the block 
        LiftHeight =    20;     % mm height of via above loading bay
        LiftTime =      0.5;    % time to "pick up" the block (v1 from LB)
        ApproachTime =  0.5;    % time to "approach" the tower (v3 from v2)
        
        % A very basic "approach position" in the frame of the block
%         ApproachP = [-jBlock.Length; 0; jBlock.Height;];
        ApproachP = [0; 0; jBlock.Height;];
        
    end 
    properties(Access = private)
        block           % defines the block to be placed in the tower 
    end
    
    methods 
        function obj = moveBlock_trj(loadBay, t0, ts, length, block)
            
            % times at all locations
            t = moveBlock_trj.simpleTime(t0, length);
            
            % input all x locations
            x = moveBlock_trj.simplePosition(loadBay, block);
            
            obj = obj@taskTrajectory(x, t, ts, moveBlock_trj.DOF);
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
            v1 = loadBay + [0; 0; moveBlock_trj.LiftHeight; 0; 0];
            
            dropLocation = [block.getPosition; 0] + ...
                    [0; 0; moveBlock_trj.DropHeight; 0; 0];
            
            v3 = moveBlock_trj.approachPosition(block);
            
            % Just picking halfway for now.
            % This via point can be used for path optimisation.
            % Also useful for avoiding collision
            v2 = (v1 + v3)/2 + [50;0;0;0;0]; 
            
            % place all position vectors into an array 
            x = [loadBay, v1, v2, v3, dropLocation];
        end
        
        % Determines the approach position based on the block position
        % And orientation
        function xApp = approachPosition(block)
            blockPos = block.getPosition;
            % rotate offset for block orientation
            P = moveBlock_trj.zRotation(blockPos(4))*...
                    moveBlock_trj.ApproachP;
                
            xApp = blockPos + [P; 0];
            xApp = [xApp; 0];
        end
        
        %Just a z rotation, nothing to see here folks
        function R = zRotation(theta)
            R = [cosd(theta) sind(theta) 0;
                -sind(theta) cosd(theta) 0;
                 0           0           1];
        end
    end
    
end
    
classdef moveBlock_trj < taskTrajectory
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Defines a trajectory to move the block from the given Loading
    bay, to drop location
    %}
    properties(Constant)
        DOF = 5;
        DropHeight =    5;      % mm drop for the block 
        LiftHeight =    20;     % mm height of via above loading bay
        LiftTime =      0.5;    % time to "pick up" the block (v1 from LB)
        ApproachTime =  0.5;    % time to "approach" the tower (v3 from v2)
        
        % A very basic "approach position" in the frame of the block
        ApproachP = [-jTower.Width; 0; 10;];
        
    end 
    properties(Access = private)
        block           % defines the block to be placed in the tower 
    end
    
    methods 
        function obj = moveBlock_trj(loadBay, t0, ts, length, block)
            
            % times at all locations
            t = simpleTime(t0, length);
            
            % input all x locations
            x = [loadBay, via_1, via_2, via_3, dropLocation];
            
            obj = obj@taskTrajectory(x, t, ts, moveBlock.DOF);
            obj.block = block;
        end
   
    end
    
    methods(Access = Private, Static)
       % Simple determination of trajectory times
        function t = simpleTime(t0, length)
            t1 = t0 + moveBlock_try.LiftTime;
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
            
            
            % place all position vectors into an array 
            x = [LoadingBay, v1, v2, v3, dropLocation];
        end
        
        function xApp = apprachPosition(block)
            blockPos = block.getPosition;
            % rotate offset for block orientation
            P = zRotation(blockPos(4))*moveBlock_trj; 
            xApp = blockPos + [P; 0; 0];
        end
        
        %Just a z rotation, nothing to see here folks
        function R = zRotation(theta)
            R = [cosd(theta) sind(theta) 0;
                -sind(theta) cosd(theta) 0;
                 0           0           1];
        end
    end
    
end
    
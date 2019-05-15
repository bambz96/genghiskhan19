classdef return_trj < taskTrajectory
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
        DOF = 5;
        DropHeight =    5;      % mm drop for the block 
        LiftHeight =    20;     % mm height of via above loading bay perhaps change this...
        ApproachTime =  0.5;    % time to "Approach" the new block (vf from v3)
        WithdrawTime =  0.5;    % time to "withdraw from" the tower (v1 from drop location)
        
        % A very basic "approach position" in the frame of the block
        ApproachP = [0; 0; jBlock.Height;]
        OpenGrip = 0.422257077; 
        
    end 
    properties(Access = private)
        block           % defines the block that has just been placed in the tower
    end
    
    methods 
        function obj = return_trj(loadBay, t0, ts, length, block)
            
            % times at all locations
            t = return_trj.simpleTime(t0, length);
            
            % input all x locations
            x = return_trj.simplePosition(loadBay, block);
            
            obj = obj@taskTrajectory(x, t, ts, return_trj.DOF);
            obj.block = block;
        end
   
    end
    
    methods(Static, Access = private)
        
        % Simple determination of trajectory times
        function t = simpleTime(t0, length)
            t1 = t0 + return_trj.WithdrawTime;
            tf = t0 + length;
            t3 = tf - return_trj.ApproachTime;
            t2 = (t1 + t3)/2;        % midpoint
            t = [t0, t1, t2, t3, tf];
        end
        
        % Simple determination of via locations
        function x = simplePosition(loadBay, block)
            dropLocation = [block.getPosition; 0] + ...
                    [0; 0; return_trj.DropHeight; 0; return_trj.OpenGrip];
                
            v1 = return_trj.approachPosition(block);
            
            v3 = loadBay + [0; 0; return_trj.LiftHeight; 0; return_trj.OpenGrip];
            % Just picking halfway for now.
            % This via point can be used for path optimisation.
            % Also useful for avoiding collision
            v2 = (v1 + v3)/2 + [50;0;0;0;return_trj.OpenGrip]; 
                        
            v0 = [loadBay(1:4); return_trj.OpenGrip];
            
            % place all position vectors into an array 
            x = [dropLocation, v1, v2, v3, v0];
        end
        
        % Determines the approach position based on the block position
        % And orientation
        function xApp = approachPosition(block)
            blockPos = block.getPosition;
            % rotate offset for block orientation
            P = return_trj.zRotation(blockPos(4))*...
                    return_trj.ApproachP;
                
            xApp = blockPos + [P; 0];
            xApp = [xApp; return_trj.OpenGrip];
        end
        
        %Just a z rotation, nothing to see here folks
        function R = zRotation(theta)
            R = [cosd(theta) sind(theta) 0;
                -sind(theta) cosd(theta) 0;
                 0           0           1];
        end
    end
    
end
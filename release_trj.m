classdef release_trj < robot_trj
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Defines a trajectory for robot to release the brick
    
    Simple form gripping:
    -moves gripper, does not move any other DOF
    -takes input coordinate, keeps this constant
    -this is pretty much the reverse of grip_trj

    %}
    properties(Constant)
        ReleaseTime = 1;  % time to open gripper NO LONGER USED
        % this is now supplied as a constructor argument
    end
    
    methods 
        function obj = release_trj(block, t, T)
            tRelease = [t, t + T];
            
            dropLocation = block.dropLocation;
            
            xGripped = [dropLocation; robot_trj.ClosedGrip];
            
            xDropped = [dropLocation; robot_trj.ClearGrip];
            
            xRelease = [xGripped, xDropped];
            
            obj = obj@robot_trj(xRelease, tRelease);
            
        end
   
    end
    
end
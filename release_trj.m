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
        PauseTime = 0.5;  % fractional time to pause 
    end
    
    methods 
        function obj = release_trj(block, t, T)
            tPause = t + release_trj.PauseTime*T;
            
            tRelease = [t, tPause, t + T];
            
            dropLocation = block.dropLocation;
            
            xGripped = [dropLocation; robot_trj.ClosedGrip];
            
            xDropped = [dropLocation; robot_trj.ClearGrip];
            
            xRelease = [xGripped, xGripped, xDropped];
            
            obj = obj@robot_trj(xRelease, tRelease);
            
        end
   
    end
    
end
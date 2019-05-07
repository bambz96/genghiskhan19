classdef pause_trj < taskTrajectory
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Defines a trajectory object for a "pause", where the robot is 
    stationary for the specified time.
   
    So simple, it's mostly just exists for the name
    %}
    
    
    properties(Access = private)
        pauseLength 
    end
    
    methods 
        function obj = pause_trj(x, t, ts, dof, length)
            tPause = [t, t + length];
            xPause = [x, x];
            obj = obj@taskTrajectory(xPause, tPause, ts, dof);
            obj.pauseLength = length;
            
        end
   
    end
    
end
    
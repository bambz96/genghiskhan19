classdef pause_trj < taskTrajectory
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Defines a trajectory object for a "pause", where the robot is 
    stationary for the specified time.
   
    So simple, it's mostly just exists for the name
    %}
    properties(Constant)
        DOF = 5;
    end
    properties(Access = private)
        pauseLength 
    end
    
    methods 
        function obj = pause_trj(x, t, ts, length)
            tPause = [t, t + length];
            xPause = [x, x];
            
            obj = obj@taskTrajectory(xPause, tPause, ts, pause_trj.DOF);
            obj.pauseLength = length;
            
        end
   
    end
    
end
    
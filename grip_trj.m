classdef grip_trj < robot_trj
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Defines a trajectory for robot "gripping"
    
    Simple form gripping:
    -moves gripper, does not move any other DOF
    -takes input coordinate, keeps this constant

    %}
    properties(Constant)
        GripTime = 1; % time to close gripper NO LONGER USED
        % this is now supplied as a constructor argument
        % currently using a very conservative value
    end
    
    methods 
        function obj = grip_trj(x, t, T)
            tGrip = [t, t + T];
            xOpenGrip = x;
            xClosedGrip = x;
            xOpenGrip(grip_trj.Gripper) = grip_trj.OpenGrip;
            xClosedGrip(grip_trj.Gripper) = grip_trj.ClosedGrip;
            xGrip = [xOpenGrip, xClosedGrip];
            
            obj = obj@robot_trj(xGrip, tGrip);
            
        end
   
    end
    
end
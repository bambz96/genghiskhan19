classdef grip_trj < taskTrajectory
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Defines a trajectory for robot "gripping"
    
    Simple form gripping:
    -moves gripper, does not move any other DOF
    -takes input coordinate, keeps this constant

    %}
    properties(Constant)
        DOF = 5;
        Gripper = 5;    % Coordinate corresponding to gripper
        OpenGrip = 0.422257077;       % Placeholder value only!!!
        ClosedGrip = 1.015976119;     % Placeholder value only!!!
        GripTime = 0.5; % time to close gripper
        % currently using a very conservative value
    end
    
    methods 
        function obj = grip_trj(x, t, ts)
            tGrip = [t, t + grip_trj.GripTime];
            xOpenGrip = x;
            xClosedGrip = x;
            xOpenGrip(grip_trj.Gripper) = grip_trj.OpenGrip;
            xClosedGrip(grip_trj.Gripper) = grip_trj.ClosedGrip;
            xGrip = [xOpenGrip, xClosedGrip];
            
            obj = obj@taskTrajectory(xGrip, tGrip, ts, grip_trj.DOF);
            
        end
   
    end
    
end
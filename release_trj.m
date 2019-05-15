classdef release_trj < taskTrajectory
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
        DOF = 5;
        Gripper = 5;        % Coordinate corresponding to gripper
        OpenGrip = 0.422257077;       % Placeholder value only!!!
        ClosedGrip = 1.015976119;     % Placeholder value only!!!
        ReleaseTime = 1;  % time to open gripper
    end
    
    methods 
        function obj = release_trj(x, t, ts)
            tRelease = [t, t + release_trj.ReleaseTime];
            xOpenGrip = x;
            xClosedGrip = x;
            xOpenGrip(release_trj.Gripper) = release_trj.OpenGrip;
            xClosedGrip(release_trj.Gripper) = release_trj.ClosedGrip;
            xRelease = [xClosedGrip, xOpenGrip];
            
            obj = obj@taskTrajectory(xRelease, tRelease, ts, grip_trj.DOF);
            
        end
   
    end
    
end
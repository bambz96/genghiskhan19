classdef robot
    %ROBOT Functions and properties of the robot 
    %   Detailed explanation goes here
    
    properties      
        %% Link Lengths
        L1      = 200;   % mm        % Origin to joint 1
        L2      = 200;	 % mm        % Joint 1+2 to Joint 3
        L3      = 200;	 % mm        % Joint 3 to Joint 4
        L4      = 56;	 % mm        % Joint 4 to Joint 5
        LE      = 82;	 % mm        % Joint 5 to End Effector
   
        %% Home Position offset
        q1_O    = 0;    % deg       % q1 offset
        q2_O    = 90;   % deg       % q2 offset
        q3_O    = -90;  % deg       % q2 offset
        q4_O    = 0;    % deg       % q2 offset
        q5_O    = 0;    % deg       % q2 offset
        
        %% Joint limits
        j1_lim = [-90 90];
        j2_lim = [-90 30];
        j3_lim = [-60 60];
        j4_lim = [-85 180];
        j5_lim = [-180 180];
        
        %% 
        forwardKinematics
        
        %% Mass Properties

        %% Motor Properties

    end
    
    methods
        function obj = robot()
            obj.forwardKinematics = forwardKinematics(obj); % Initialise Forward Kinematics
        end
        
        function drawRobot = drawPose(obj,q1,q2,q3,q4,q5)
            [x,y,z] = obj.forwardKinematics.findCoordinates(q1,q2,q3,q4,q5);
            drawRobot = roboDraw(x,y,z);
        end
        
    end
   
            
    
end


classdef robot
    %ROBOT Functions and properties of the robot
    %   Detailed explanation goes here

    properties
        DoF = 5; %Degrees of Freedom


        %% Link Lengths
        L1      = 200;   % mm        % Origin to joint 1
        L2      = 200;	 % mm        % Joint 1+2 to Joint 3
        L3      = 200;	 % mm        % Joint 3 to Joint 4
        L4      = 100;	 % mm        % Joint 4 to End effector

%         L1 = sym('L1');
%         L2 = sym('L2');
%         L3 = sym('L3');
%         L4 = sym('L4');

        %% Home Position offset
        q1_O    = 0;    % deg       % q1 offset
        q2_O    = 90;   % deg       % q2 offset
        q3_O    = -90;  % deg       % q2 offset
        q4_O    = 0;    % deg       % q2 offset
        q5_O    = 0;    % deg       % q2 offset

        %% Joint limits
        j1_lim = [-90 90]
        j2_lim = [-100 89]
        j3_lim = [-63 54]
        j4_lim = [-20 150]
        j5_lim = [-112 112]
        
        %% Base Limits
        % Keep out zone definitions for cuboid
        % LFR, LFL, LRL, LRR, UFR, UFL, URL, URR
        baseX = [78 78  -78 -78 78  78  -78 -78]
        baseY = [78 -78 -78 78  78  -78 -78 78]
        baseZ = [0  0   0   0   300 300 300 300]
        %% End Effector Dimensions
        length = 120    %mm
        width = 66      %mm

        %%
        forwardKinematics
        inverseKinematics
        differentialKinematics

        %% Mass Properties

        %% Motor Properties

    end

    methods
        function obj = robot()
            obj.forwardKinematics = forwardKinematics(obj); % Initialise Forward Kinematics
            obj.inverseKinematics = inverseKinematics(obj.forwardKinematics.T_0E, obj); % Initialise Inverse Kinematics
            obj.differentialKinematics = differentialKinematics(obj);
        end

        function drawRobot = drawPose(obj,q1,q2,q3,q4,q5)
            [x,y,z] = obj.forwardKinematics.findCoordinates(q1,q2,q3,q4,q5);
            drawRobot = roboDraw(x,y,z);
        end

    end


end

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
        L_PL    = 100;   % mm        % Shorter side length of Parallelogram


%         L1 = sym('L1');
%         L2 = sym('L2');
%         L3 = sym('L3');
%         L4 = sym('L4');

        %% Home Position offset     
        q1_O    = 0; % deg       % q1 offset
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
        % Keep out zone definitions for base
        baseWidth = 155
        baseLength = 155
        baseHeight = 300
        baseOrientation = 0
        baseX = 0
        baseY = 0
        %% End Effector Dimensions
        EE_length = 120    %mm
        EE_width = 66      %mm

        %%
        forwardKinematics
        inverseKinematics
        differentialKinematics

        %% Mass Properties
        g           = 9.81;         % m/s^2 % Gravity
        m_total     = 0.764;        % kg    % Mass of all moving parts % MEASURE AT SOME POINT
        m_PL_bot    = 0.085;        % kg    % Mass of lower parallelogram joint
        m_PL_top    = 0.075;        % kg    % Mass of upper parallelogram joint
        m_counter   = 0;            % kg    % Mass of counterweight
        m_3         = 0.054;        % kg    % Mass of joint 3
        m_4         = 0.063;        % kg    % Mass of joint 4 (before EE)
        m_E         = 0.116;        % kg    % Mass of End Effector
        
        %% Other Properties
        tSpring_K   = 7.25;           % Nmm/deg % Stiffness of torsion spring
        tSpring_0   = 0;            % deg     % Zero position of torsion spring

        %% Motor Properties
        XL430_T_Max = 0.84;     %Nm
        XL320_T_Max = 0.22;     %Nm

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

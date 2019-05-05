classdef robot < handle
    %ROBOT Functions and properties of the robot 
    %   Detailed explanation goes here
    
    properties      
        DoF = 5; %Degrees of Freedom

        %% Link Lengths
        L1      = 206.25;   % mm        % Origin to joint 1
        L2      = 200;	 % mm        % Joint 1+2 to Joint 3
        L3      = 200;	 % mm        % Joint 3 to Joint 4
        L4      = 100;	 % mm        % Joint 4 to End effector
        L_PL    = 100;   % mm        % Shorter side length of Parallelogram
   
        %% Home Position offset
        q1_O    = 0;    % deg       % q1 offset
        q2_O    = 90;   % deg       % q2 offset
        q3_O    = -90;  % deg       % q2 offset
        q4_O    = 0;    % deg       % q2 offset
        q5_O    = 0;    % deg       % q2 offset
        
        %% Joint limits
        j1_lim  = [-90 90];
        j2_lim  = [-90 30];
        j3_lim  = [-60 60];
        j4_lim  = [-85 180];
        j5_lim  = [-180 180];
        
        %% Joint Speed Limits
        j1_speed = 5; % RPM
        j2_speed = 5; % RPM
        j3_speed = 5; % RPM
        j4_speed = 5; % RPM
        j5_speed = 5; % RPM

        %% Object Properties
        forwardKinematics       % Forward Kinematics Calcs
        inverseKinematics       % Inverse Kinematics Calcs
        differentialKinematics  % Jacobian, inv Jacobian and velocity calcs
        motorControl            % Motor instantiation and functionality
        torque                  % Motor torque calculations
        trajectoryPlanning      % Plan and generate trajectories
        
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
        tSpring_K   = 10;           % Nmm/deg % Stiffness of torsion spring
        tSpring_0   = 0;            % deg     % Zero position of torsion spring

    end
    
    methods
        function obj = robot()
            obj.forwardKinematics = forwardKinematics(obj); % Initialise Forward Kinematics
            obj.inverseKinematics = inverseKinematics(obj.forwardKinematics.T_0E, obj); % Initialise Inverse Kinematics
            obj.motorControl = motorControl();
            obj.torque = torque(obj); 
            obj.differentialKinematics = differentialKinematics(obj);
            obj.trajectoryPlanning = trajectoryPlanning(obj.inverseKinematics, obj.differentialKinematics);
        end
        
        function drawRobot = drawPose(obj,q1,q2,q3,q4,q5)
            [x,y,z] = obj.forwardKinematics.findCoordinates(q1,q2,q3,q4,q5);
            drawRobot = roboDraw(x,y,z);
        end
        
        function enableMotorTorques(obj)
            obj.motorControl.enableTorque;
        end
        
        function disableMotorTorques(obj)
            obj.motorControl.disableTorque;
        end
        
        function setRobotJoints(obj,q1,q2,q3,q4,q5)
            % absolute angles
            obj.motorControl.setAngles(q1,q2,q3,q4,q5,obj.j1_speed);
        end
        
        function setRobotPose(obj,x,y,z,theta)
            [q1,q2,q3,q4,q5] = obj.inverseKinematics.findQ(x,y,z,theta);
            obj.motorControl.setAngles(q1,q2,q3,q4,q5,obj.j1_speed);
        end
        
        function [x,y,z,theta] = getRobotPose(obj)
            [q1,q2,q3,q4,q5] = obj.motorControl.getAngles();
            [x,y,z,theta] = obj.forwardKinematics.getPose(q1,q2,q3,q4,q5);
        end
        
        function homePosition(obj)
            obj.setRobotJoints(0,0,0,0,0);
        end

    end
   
end


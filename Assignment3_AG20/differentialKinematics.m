classdef differentialKinematics <handle

%     Differential Kinematics for robot class, calculates robot jacobian
%
%     Usage: class should be initialised in the constructor of the robot to
%     which these differentaial kinematics are assigned.
%
%     Currently just calculates jacobian, additional functionality to be
%     added in future release.



   %%  Properties
    properties
        Jacobian
        Jacobian_func
        invJacobian
        Frame = 'W';
        robot

        z_hat = [0;0;1];

        %% Symbolic Joint States
        q1      = sym('q1'); % deg
        q2      = sym('q2'); % deg
        q3      = sym('q3'); % deg
        q4      = sym('q4'); % deg
        q5      = sym('q5'); % deg
    end

    %% Methods
    methods
        %Constructor
        function obj = differentialKinematics(robot)
            obj.Jacobian = jacobianCalculator(obj, robot);
            obj.robot = robot;
%            obj.invJacobian = invJacobianCalculator(obj.Jacobian);
        end

        %Jacobian calculator (most important function)
        function Jacobian = jacobianCalculator(obj, robot)
            Jacobian = [];
            for joint = 1:(robot.DoF)
                R = obj.Rotation(robot, 0, joint);
                z = R*obj.z_hat;
                P = R*obj.Position(robot, joint, obj.Frame);

                Jacobian = [Jacobian, obj.jacobianRevolute(z, P)];

            end
            %Final Result
%             Jacobian = simplify(Jacobian);
            obj.Jacobian_func = matlabFunction(Jacobian);
        end

        % Find numerical Jacobian
        function JacobianN = evalJacobian(obj,q1,q2,q3,q4,~)
            JacobianN = obj.Jacobian_func(q1,q2,q3,q4);
%             % Replace symbols with values
%             JacobianN = subs(obj.Jacobian,[obj.q1,obj.q2,obj.q3,obj.q4,obj.q5],[q1,q2,q3,q4,q5]);
%             % Convert to numeric value
%             JacobianN = double(JacobianN);
        end

        % Calculate Task Space Velocities
        function [x_dot_i] = findTaskSpaceVelocities(obj,q_i,q_dot_i)
            % x_dot_i: 6x1 matrix of end effector velocities in frame 0
            %          First three elements are in mm/s (x_dot,y_dot,z_dot)
            %          elements 4,5,6 are in deg/s
            % q_dot: 1x5 matrix of joint positions in degrees
            % q_dot_i: 5x1 matrix of joint velocities in degrees/s
            degreesToRads = pi/180;
            JacobianEval = evalJacobian(obj,q_i(1), q_i(2), q_i(3), q_i(4), q_i(5));
            x_dot_i = JacobianEval*q_dot_i;
            for i = 1:3
                x_dot_i(i) = x_dot_i(i)*degreesToRads;
            end
        end

        % Calculate joint velocities based on task space velocities
        function [q_dot_i] = findJointSpaceVelocities(obj,q_i,x_dot_0_i)
            % q_dot_i: 5x1 matrix of joint velocities in deg/s
            % x_dot_0_i: 6x1 matrix of end effector velocities in frame 0
            % in mm/s (linear velocities) and deg/s (angular velocities)
            % q_i: 5x1 matrix of joint positions in degrees
            radsToDegrees = 180/pi;

            % Method 2 taught in lecture 19
            % Express everything in Frame 1
            J_N = evalJacobian(obj,q_i(1),q_i(2),q_i(3),q_i(4),q_i(5));

%             R_01_f = matlabFunction(RotationFast(obj,obj.robot,1,0));
%             R_01 = R_01_f(q_i(1));

            R_01 = double(subs(RotationFast(obj,obj.robot,1,0),obj.q1,q_i(1)));


            x_dot_1_i = [R_01 zeros(3,3);
                        zeros(3,3) R_01]*x_dot_0_i;
            J_1 = [R_01 zeros(3,3);
                    zeros(3,3) R_01]*J_N;

            % Remove 4th column, as there can't be a rotation about X_1
            x_dot_1_i(4) = [];
            J_1(4,:) = [];

            % Multiply in conversion constant to first three elements of
            % x_dot so that when q_dots are calculated, output is in deg/s
            for i = 1:3
                x_dot_1_i(i) = x_dot_1_i(i)*radsToDegrees;
            end

            % Invert Jacobian
%             J_1_Inv = inv(J_1N);

            % Calculate q_dot_i by inverting J_1
            q_dot_i = J_1\x_dot_1_i;
        end


        %Helper function for calculating jacobian for a single revolute
        %joint
        function Ji = jacobianRevolute(obj, zi, Pi)
            Ji = cross(zi, Pi); %linear velocity
            Ji= [Ji;zi];        %angular velocity
        end


        %%  Wrapper functions
        %Wrapper function for a method called on robot.forwardKinematics
        function R = Rotation(obj, robot, frameExpressed, frameOf)
            R = robot.forwardKinematics.getRotation(frameExpressed, frameOf);
        end

        %Wrapper function for a method called on robot.forwardKinematics
        function R = RotationFast(obj, robot, frameExpressed, frameOf)
            R = robot.forwardKinematics.getRotationFast(frameExpressed, frameOf);
        end

        %Wrapper function for a method called on robot.forwardKinematics
        function P = Position(obj, robot, frameExpressed, frameOf)
            P = robot.forwardKinematics.getPosition(frameExpressed, frameOf);
        end

    end

end

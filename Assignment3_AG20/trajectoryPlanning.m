classdef trajectoryPlanning < handle
    %TRAJECTORYPLANNING Generates Trajectories for robot motion
    %   Detailed explanation goes here
    
    properties
        inverseKinematics
        differentialKinematics
    end
    
    methods
        function obj = trajectoryPlanning(inverseKinematics, differentialKinematics)
            %TRAJECTORYPLANNING Construct an instance of this class
            %   Detailed explanation goes here
            obj.inverseKinematics = inverseKinematics;
            obj.differentialKinematics = differentialKinematics;
            % Generate trajectory to go home from current position
        end
        
        function [t, x, x_dot, x_double_dot] = generateTaskTrajectory(obj, x0,x_dot_0,xf,x_dot_f, tf, dt)
            % Generates trajectory in task space x: x,y,z,theta in frame 0
            % x0:       start position 1x4 (x,y,z,theta)
            % x_dot_0:  start velocities 1x4 (x_dot,y_dot,z_dot,theta_dot)
            % xf:       end position 1x4 (x,y,z,theta)
            % x_dot_f:  end velocities 1x4 (x_dot,y_dot,z_dot,theta_dot)
            % tf:       time taken to complete trajectory
            % dt:       time step (resolution of discretisation) 
            
            % t: sx1 vector of time values
            % x: sxn matrix of positions
            % x_dot: sxn matrix of velocities
            % x_double_dot: sxn matrix of accelerations
            
            %% Preallocation of matrices
            s = ceil(tf/dt); % Number of steps in each time series
            n = length(x0); % Number of coordinates
            t = zeros(s,1); % preallocating nx1 column vector of time values
            for k = 1:s
                t(k) = k*dt;
            end
            x = zeros(s,n); % Preallocating position matrix
            x_dot = zeros(s,n); % Preallocating velocity matrix
            x_double_dot = zeros(s,n); % Preallocating acceleration matrix
            
            %% Setting up matrices for solving coefficients
            A = [1 0    0       0;
                 0 1    0       0; 
                 1 tf   tf^2    tf^3;
                 0 1    2*tf    3*tf^2];
            
            a = zeros(4,n); % Matrix of coefficients
            % x(t) = a3*t^3 + a2*t^2 + a1*t + a0
            % a(1) = a0, a(2) = a1, a(3) = a2, a(4) = a3
            
            x_known = [ x0;
                        x_dot_0;
                        xf;
                        x_dot_f];
            
            %% Calculating time series'
            for i = 1:n
                a(:,i) = A\x_known(:,i); % Solve for coefficients
                x_func = @(t)       a(4,i)*t^3+   a(3,i)*t^2+   a(2,i)*t+   a(1,i);
                x_dot_func = @(t)               3*a(4,i)*t^2+ 2*a(3,i)*t+   a(2,i); 
                x_double_dot_func = @(t)                      6*a(4,i)*t+ 2*a(3,i);
                for j = 1:s 
                    x(j,i) = x_func(t(j,1));
                    x_dot(j,i) = x_dot_func(t(j,1));
                    x_double_dot(j,i) = x_double_dot_func(t(j,1));
                end
            end
        end
        
        function [t, q, q_dot] = generateJointTrajectory(obj, t_in, x, x_dot)
            t = t_in; 
            s = length(t);
            n = 5; % Number of generalised coordinates
            
            %% Preallocating matrices
            q = zeros(s,n);
            q_dot = zeros(s,n); 
            % q_double_dot = zeros(s,n); % To be implemented in the future
            
            for i = 1:s
                [q1, q2, q3, q4, q5] = obj.inverseKinematics.findQ(x(i,1),x(i,2),x(i,3),x(i,4));
                q_i = [q1;
                        q2;
                        q3;
                        q4;
                        q5];
                x_dot_0_i = [x_dot(i,1);
                             x_dot(i,2);
                             x_dot(i,3);
                             0;
                             0;
                             x_dot(i,4)];
                q_dot_i = obj.differentialKinematics.findJointSpaceVelocities(q_i,x_dot_0_i);
                
                q(i,:) = q_i';
                q_dot(i,:) = q_dot_i';
                %q_double_dot = ... % Future implementation
            end
        end
    end
    
    methods(Static)
        function plotTrajectories(t,x,x_dot,x_double_dot)
            m = length(x(1,:)); % Number of coordinates
            n = 3; % Three types of graph, x, x_dot, x_double_dot
            figure
            for i = 1:m
                for j = 1:n
                    k = (i-1)*n+j; % Plot number
                    subplot(m,n,k);
                    if j == 1
                        plot(t,x(:,i))
                    elseif j == 2
                        plot(t,x_dot(:,i))
                    elseif j == 3
                        plot(t,x_double_dot(:,i))
                    end
                end
            end
        end
    end
end


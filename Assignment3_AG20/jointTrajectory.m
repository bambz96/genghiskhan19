classdef jointTrajectory
    %{
        Assignemnt Group 20
        Joint Trajectory class, stores a trajectory in joint space, 
        
        Class is instatniated from a task space trajectory
    
        Question: is there any good reason to use inheritance here?
        IE: should we make an abstract class for both jointTrajectory and 
        taskTrajectory to inherit from?
        Answer this later
            
    %}
    %% Propperties
    properties (Access = private)
        ts      % sampling time
        dof     % Degrees of Freedom (Number of joints)
        t       % Times for each position (start, end, and each via)
        
        pieces  % number of pieces in the trajectory
        
        Q       % Stores the joint space trajectory
        Q_dot   % Stores the joint velocity profile
        Q_ddot  % Stores the joint acceleration profile 
        
        time    % timeseries associated with trjectory
        
    end
    
   %% Constructor
   
    methods
        % Note, jointTrajectory must be initialised from a taskTrajectory
        function obj = jointTrajectory(taskT, dof, IK, DK) % probs DK too
            % Input propperties
            obj.dof = dof;
            
            % Propperties taken directly from taskTrajectory
            [obj.t, obj.ts] = taskT.getTime;
            obj.time = taskT.getTimeseries;
            
            obj.pieces = length(obj.t) - 1;
            
            % Propperties calculated based on taskTrajectory
            obj.Q = obj.findJointTrajectories(taskT.getPosition, IK);
            obj.Q_dot = obj.findJointVelocities(taskT.getPosition, DK);
            
        end
    
        %% Accessors
        function Q = getQ(obj)
            Q = obj.Q;
        end
        
        function T = getTimeseries(obj)
            T = obj.time;
        end
        
        function Qd = getQ_dot(obj)
            Qd = obj.Q_dot;
        end
        
        function Qdd = getQ_ddot(obj)
            Qdd = obj.Q_ddot;
        end
    
    end
    
    %% Private Methods
    
    methods(Access = private)
        
        % Finds joint trajectories from task trajectories
        function Q = findJointTrajectories(obj, X, IK)
            n = length(obj.time);
            %Pre-allocate arrays for joint trajectories 
            Q = zeros(obj.dof, n);
            for i = 1:n
                [Q(1,i), Q(2,i), Q(3,i), Q(4,i), Q(5,i)] = ...
                    IK.findQ(X(1,i), X(2,i), X(3,i), X(4,i)); 
            end
            % Done?
        end
        

        % Finds joint velocities from joint trajectories, and task
        % velocities
        function Qd = findJointVelocities(obj, X, DK)
            n = length(obj.time);
            Qd = zeros(obj.dof, n);
            for i = 1:n
                % length 6 vector for differential kinemantics
                x6 = [X(1, i);
                    X(2,i);
                    X(3,i);
                    0; 
                    0; 
                    X(4, i)]; 
                Qd(:,i) = DK.findJointSpaceVelocities(obj.Q(:,i), x6);
            end
        end
    end
    
    
 end

    
    
    
    
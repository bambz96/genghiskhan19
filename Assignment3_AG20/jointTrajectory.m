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
        function obj = jointTrajectory(taskT, dof, IK) % probs DK too
            % Input propperties
            obj.dof = dof;
            
            % Propperties taken directly from taskTrajectory
            [obj.t, obj.ts] = taskT.getTime;
            obj.time = taskT.getTimeseries;
            
            obj.pieces = length(obj.t) - 1;
            
            % Propperties calculated based on taskTrajectory
            obj.Q = obj.findJointTrajectories(taskT.getPosition, IK);
            
        end
    
        %% Accessors
        function Q = getQ(obj)
            Q = obj.Q;
        end
        
        function T = getTimeseries(obj)
            T = obj.time;
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
        

        
%         function Q_dot = findJointVelocities(obj, X)
% 
%         end
        
    end
    
 end

    
    
    
    
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
    
   %% Methods
   
    methods
        % Note, jointTrajectory must be initialised from a taskTrajectory
        function obj = jointTrajectory(taskT, dof)
            % input propperties
            obj.dof = dof;
            
            % propperties taken directly from taskTrajectory
            [obj.t, obj.ts] = taskT.getTime;
            obj.time = taskT.getTimeseries;
            
            obj.pieces = length(obj.t) - 1;
            
            %Propperties calculated based on taskTrajectory
            
            
        
        
        end
        
        
        
    end
    
end
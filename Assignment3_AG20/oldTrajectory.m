classdef oldTrajectory
    %{
        Assignment Gropup 20
        Trajecotry class: stores a timeseries and trajectory 
    
        OK, everything working, except velocities, when ther eare multiple
        degrees of Freedom. 
        
        Put this one asside for now...
    
    %}
    
    properties %(Access = private)
        ts      % sampling time
        dof     % Degrees of Freedom in the trajectory
        x       % positions specified including start, end, and vias
        t       % associated times for each given position
        
        Vi      % Initial velocity vector -assumed to be zero
        Vf      % Final velocity vector -assumed to be zero
        
        X       % Stores the trajectory
        V       % Stores the associated velocity profile
        time    % Timeseries associated with trajectory 
        PPoly   % Piecewise polynomial form of the trajectory
    end 
    
    methods 
        %constructor
        function obj = oldTrajectory(x, t, ts, dof)
        obj.x = x;
        obj.t = t; 
        obj.ts = ts;
        obj.dof = dof;
        
        obj.Vi = zeros(dof,1); % assumed zero
        obj.Vf = zeros(dof,1); % assumed zero
        obj.time = t(1):ts:t(end);
        
        % calculates the trajectory as a piecwise polynomial
        obj.PPoly = spline(t, [obj.Vi, x, obj.Vf]); 
        
        % generates the position trajectory vector
        obj.X = ppval(obj.PPoly, obj.time); 
        obj.V = velocityCalc(obj);
        end
        
        
        %some accessors
        function T = getTimeseries(obj)
            T = obj.time;
        end
        
        function X = getPosition(obj)
            X = obj.X;
        end
        
%         This bit is still buggy
        function V = getVelocity(obj)
            V = obj.V;
        end
        
        function C = getCoefficients(obj)
            C = obj.PPoly.coefs;
        end
        
        
    end
    
    
    % Some private helper methods
    methods(Access = private)
        
        function V = velocityCalc(obj)
            % Create Polynomial Differentiation operator
            D = diag((obj.PPoly.order -1):-1:1, 1);
            % Differentiate Polynomial Coefficients
            VCoeffs = obj.getCoefficients*D;
            
            % Make a piecewise polynomial of Velocity profile
            VPP = mkpp(obj.PPoly.breaks, VCoeffs);
            % Create velocity profile
            V = ppval(VPP, obj.time);
        end
        
        
    end
    
end
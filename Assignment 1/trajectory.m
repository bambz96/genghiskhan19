classdef trajectory
    %{
        Assignment Gropup 20
        Trajecotry class: stores a timeseries and trajectory 
    
        Going to completely rework this class to do things manually...
    
    %}
    properties (Constant)
       order = 3;                       % order of polynomials, ie: cubic 
    end
    
    properties (Access = private)
        ts      % sampling time
        dof     % Degrees of Freedom in the trajectory
        x       % positions specified including start, end, and vias
        t       % associated times for each given position
        
        pieces  % number of pieces in the trajectory
        nCons   % number of constraints
        
        Coefs   % Stores trajectory coefficients 
        X       % Stores the trajectory
        V       % Stores the associated velocity profile
        time    % Timeseries associated with trajectory 

        
    end 
    
    methods 
        %constructor
        function obj = trajectory(x, t, ts, dof)
        obj.x = x;
        obj.t = t; 
        obj.ts = ts;
        obj.dof = dof;
        
        % number of pieces in the plynomial
        obj.pieces = length(x) - 1;
        % number of constraints required 
        obj.nCons = (obj.order + 1)*obj.pieces;
        
        
        obj.time = t(1):ts:t(end);
        
        
        % calculates the trajectory as a piecwise polynomial
        obj.Coefs = obj.calculateCoefficients;
        
        % generates the position trajectory vector
        obj.X = obj.calculateTrajectory;
        
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
            C = obj.Coefs;
        end
        
        
    end
    
    
    % Some private helper methods
    methods (Access = private)
        
        function Traj = calculateTrajectory(obj)
            Traj = obj.x(1); 
            for p = 1:obj.pieces
                tt = (obj.t(p) + obj.ts):obj.ts:obj.t(p + 1); %time series 
                Traj = [Traj, polyval(obj.Coefs(p,:), tt)];
            end
        end
        
        function C = calculateCoefficients(obj)
            A = obj.findA();
            B = obj.findX();
            Sol = linsolve(A, B); %solution to system of linear equations
            
            C = []; %inititalize empty array
            
            % arrange coefficients by pieces
            for piece = 1:(obj.pieces)
                C = [C; (Sol(4*piece:-1:4*(piece-1)+1))'];
            end
                
        end
        
        % probably come up with a better name than this...
        function A = findA(obj)
            % initialize A
            A = zeros(obj.nCons);
            
            % initial endpoint constraints
            A(1:2, 1:4) = [obj.constraint(obj.t(1), 0);  %position 
                           obj.constraint(obj.t(1), 1)]; %velocity
                          
            % final endpoint constraints
            A(end-1:end, end - 3:end) = ...
                [obj.constraint(obj.t(end), 0);     %position
                 obj.constraint(obj.t(end), 1)];    %velocity
             
            for knot = 1:(obj.pieces - 1)
                A(knot*4 - 1: knot*4 + 2, (knot-1)*4 + 1: (knot+1)*4) = ...
                    obj.knotConstraints(obj.t(knot + 1));
            end
             
        end
        
        
        
        % constructs the constraints around a knot
        % where tk is the time at the knot
        function KC = knotConstraints(obj, tk) 
            
            %components of knot constraints
            posC = obj.constraint(tk, 0);   % position constraint
            velC = obj.constraint(tk, 1);   % velocity constraint
            accC = obj.constraint(tk, 2);   % acceleration constraint
            zero = zeros(1, obj.order + 1); % zeros
            
            KC = [posC, zero;
                  velC, -velC;
                  zero  posC;
                  accC, -accC];
        end
            
        
        
        function C = constraint(obj, t, deriv)
            % Polynomial Differentiation operator
            D = diag(1:obj.order, 1);
            
            % position constraint
            C  = [1 t t^2 t^3];
            % differentiate if needed
            for i = 1:deriv
                C = C*D;
            end
        end
        
        
        
        % Constructs the RHS of constraints
        % I feel like this function could be better...
        function X = findX(obj)
            knot = 1; 
            after = 1;
            %initialize to zero
            X = zeros(obj.nCons, 1);
                
            for constr = 1:(obj.nCons)
                if (mod(constr, 2) ~= 0)
                    X(constr) = obj.x(knot);
                    if after
                        knot = knot + 1;
                        after = 0;
                    else
                        after = 1;
                    end
                end
            end
        end
        
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
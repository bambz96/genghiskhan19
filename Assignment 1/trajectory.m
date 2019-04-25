classdef trajectory
    %{
        Assignment Gropup 20
        Trajecotry class: stores a timeseries and trajectory 
    
        Trajectory is calculated on initialisation, from inputs to the
        class constructor
        
        Trajectory propperties shoudl be accessed using accessor methods 
        supplied.
    
    %}
    
    %% Propperties
    properties (Constant)
        order = 3;   % order of polynomials, ie: cubic 
        N = 4; %number of coefficients to describe one piece 
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
        Acc     % Acceleration Profile
        Jerk    % Jerk Profile   
        time    % Timeseries associated with trajectory 

        
    end 
    
    %% Methods (Constructor)
    methods 
        % inputs: 
        % x: array containting values for start and end points, and all
        % vias for the trajectory, if dof > 1, x should have one row for
        % each dimension
        % t: time vector correspondig to the x values given
        % ts: sample time
        % dof: degrees of freedom 
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
        
        % generate the position trajectory vector
        obj.X = obj.allTrajectories;
        
        % generate the velocity, accel, jerk profiles
        obj.V = obj.allDerivTrajectories(1);
        obj.Acc = obj.allDerivTrajectories(2);
        obj.Jerk = obj.allDerivTrajectories(3);   
        
        end
        
        
        %% Accessors for all dof
        function T = getTimeseries(obj)
            T = obj.time;
        end
        
        function X = getPosition(obj)
            X = obj.X;
        end
        
        function V = getVelocity(obj)
            V = obj.V;
        end
        
        function A = getAcceleration(obj)
            A = obj.Acc;
        end
        
        function J = getJerk(obj)
            J = obj.Jerk;
        end
        
        function C = getCoefficients(obj)
            C = obj.Coefs;
        end
        
        %% Accessors for individual variable trajectories etc.
        
        function x = getOnePosition(obj, degree)
            x = obj.X(degree,:);
        end
        
        function v = getOneVelocity(obj, degree)
            v = obj.V(degree, :);
        end
        
        function a = getOneAcceleration(obj, degree)
            a = obj.A(degree, :);
        end
        
        function j = getOneJerk(obj, degree)
            j = obj.J(degree, :);
        end
        
        function c = getOneCoefs(obj, degree)
            c = obj.Coefs(:, :, degree);
        end
        
        
    end
    
    
    % Calculation methods
    methods (Access = private)
        
        %% High level functions (For all Trajectories)
        % Would be really nice if matlab allowed higher order functions...
        
        % Calculate trajectories for all degrees of freedom
        function T = allTrajectories(obj)
            T = [];
            
            % Calculate trajectory for each degree of freedom
            for deg = 1:obj.dof
                Traj = calculateTrajectory(obj, obj.getOneCoefs(deg));
                T = [T;Traj];
            end
        end
        
        
        % Sets up systems of linear equations for each coordinate
        function C = calculateCoefficients(obj)
            % note: since, the time valuse of every via are the same, 
            % the LHS is the same for each set of equations
            LHS = obj.findLHS();
            
            C = [];
            
            for deg = 1:(obj.dof)
                % Set up RHS
                RHS = obj.findRHS(obj.x(deg,:));
                Cnew = obj.solveTrajectory(LHS, RHS);
                C = cat(3, C, Cnew);
            end
        end            
        
        function V = allDerivTrajectories(obj, deriv)
            V = [];
            
            % Calculate deriv trajectory for each degree of freedom
            for deg = 1:obj.dof
                Vel = calcDerivTrajectory(obj, deriv, obj.Coefs(:,:,deg));
                V = [V;Vel];
            end
        end
            
        
        %% Calculation methods for one variable
        
        % Calculates a trajectory for one variable
        % corresponding to the time series
        % Generalised for use with derivatives
        % input, CoEffiecients of trajectory
        function Traj = calculateTrajectory(obj, Co)
            Traj = polyval(Co(1,:), obj.t(1)); %initial value
            for p = 1:obj.pieces
                % Sequentially calculate each piece
                tt = (obj.t(p) + obj.ts):obj.ts:obj.t(p + 1); %time series 
                Traj = [Traj, polyval(Co(p,:), tt)];
            end
        end
        
        
        % Solves the system of linear equations for one trajectory
        function C = solveTrajectory(obj, A, B)

            Sol = linsolve(A, B); %solution to system of linear equations
            
            C = []; %inititalize empty array
            
            % arrange coefficients by pieces
            for piece = 1:(obj.pieces)
                C = [C; (Sol(obj.N*piece:-1:obj.N*(piece-1)+1))'];
            end
                
        end
        
        % Determines the LHS of the constraint equation
        function A = findLHS(obj)
            % initialize A to zero
            A = zeros(obj.nCons); 
            
            % initial endpoint constraints
            A(1:2, 1:4) = [obj.constraint(obj.t(1), 0);  %position 
                           obj.constraint(obj.t(1), 1)]; %velocity
                          
            % final endpoint constraints
            A(end-1:end, end - 3:end) = ...
                [obj.constraint(obj.t(end), 0);     %position
                 obj.constraint(obj.t(end), 1)];    %velocity
            
            n = obj.N; % reassigned for readability
            for knot = 1:(obj.pieces - 1)
                A(knot*n - 1:knot*n + 2, (knot-1)*n + 1:(knot+1)*n) = ...
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
            zero = zeros(1, obj.N); % zeros
            
            KC = [posC,  zero;
                  velC, -velC;
                  zero,  posC;
                  accC, -accC];
        end
            
        
        % Can construct parts of each constraint, 
        % All constraints are a function of position, velocity or
        % acceleration
        % deriv: refers to the degree of differentiation ie: position => 0,
        % velocity => 1 etc. 
        function C = constraint(obj, t, deriv)
            % Polynomial Differentiation operator
            % This operator is specific to the form shown below
            D = diag(1:obj.order, 1);
            
            % position constraint
            C  = [1 t t^2 t^3];
            % differentiate if needed
            for i = 1:deriv
                C = C*D;
            end
        end
        
        
        
        % Constructs the RHS of constraint equations
        % Used once for each coordiante
        % xVal gives the via locations of the coordinate 
        function X = findRHS(obj, xVal)
            knot = 1; 
            after = 1;
            %initialize to zero
            X = zeros(obj.nCons, 1);
                
            for constr = 1:(obj.nCons)
                if (mod(constr, 2) ~= 0) % Every seccond constraint = 0
                    X(constr) = xVal(knot); %Every other = position 
                    if after
                        knot = knot + 1;
                        after = 0;
                    else
                        after = 1;
                    end
                end
            end
        end
        

        % Calculate the velocity, acceleration, jerk profiles
        % deriv: refers to the degree of differentiation
        % Pass in Coefficients
        function V = calcDerivTrajectory(obj, deriv, Coeffs)
            % Create Polynomial Differentiation operator
            % This operator fits the form of the stored trajectory
            D = diag((obj.order):-1:1, 1);
            
            % Differentiate Polynomial Coefficients    
            for diff = 1:deriv
                Coeffs = Coeffs*D;
            end
            % Create velocity profile
            V = obj.calculateTrajectory(Coeffs);
        end
        
    end
    
end
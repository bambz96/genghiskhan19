classdef robot_trj < taskTrajectory
    %{ 
    Project Group 10
    Robotics Systems MCEN90028
    
    Robot Trajectory Class:
    Inherits from task trajectory
    Stores some propperties specific to the robot. 
    
    Includes methods for converitng trajectories into correct format 
    for the Great Jenghis Khan.
 
    
    %}
    
    properties(Constant)
        DOF = 5;                    % Degrees of Freedom
        DATACOLS = 6;               % Number of variables to send   
        % row vector [a3, a2, a1, a0, ts, tf], stores coefficients start
        % time and end time for each polynomial. 
        
        % Sampling time not actually passed to robot. 
        % This is just for simulation/plotting
        TS = 0.1;                   % Sample Time         
        
        Gripper = 5;                % Coordinate corresponding to gripper
        OpenGrip = 0.422257077;     % Radians
        ClosedGrip = 1.015976119;   % Radians
        
        Truncation = 1e-5           % Threshold for truncation of data
        
        % Note: Probably a couple more that should be added    
    end
    
    properties(Access = private)
        DATA    % Stores data to send to the Robot
        % row vector [a3, a2, a1, a0, ts, tf], stores coefficients start
        % time and end time for each polynomial. 
        % rows: Pieces
        % pages: Coordinates
    end
        
    
%     Constructor
    methods 
        function obj = robot_trj(x, t)
                        
            obj = obj@taskTrajectory(x, t, robot_trj.TS, robot_trj.DOF);
            
            obj.DATA = obj.robotDATA;
            
        end
        
        % returns trajectories for all degrees of freedom. 
        function DATA = getDATA(obj)
            DATA = obj.DATA;
        end
        
        % Returns data for one DOF
        function xDATA = getCoordinateDATA(obj, coordinate)
            xDATA = obj.DATA(:,:,coordinate);
        end
        
        
    end
    
    
    methods(Access = private)
        
        
        % Converts trajectory coefficients into the correct form to send to
        % The Great Jenghis Khan.

        function DATA = robotDATA(obj)
            
            DATA = zeros(pieces, obj.DATACOLS, obj.DOF);
            
            for x = 1:obj.DOF
                % Get Coefficients for one coordinate trajectory
                Coeffs = obj.getCoefficients;
                for p = 1:pieces
                    % shrink data
                    for a = 1:4
                        DATA(p, a, x) = myShrink(Coeffs(p, a, x));
                    end
                    DATA(p, 5:6, x) = TimeVals(p:p + 1);
                end
                
            end
            
        end        
        
        function val = myShrink(x)
            if abs(x) < 1e-5
                val = 0;
            else
                val = x;
            end
        end
        
        
    end
    
    
end

    


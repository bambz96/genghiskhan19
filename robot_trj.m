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
        
        TruncationK = 1e-5           % Threshold for truncation of data
        
        % Note: Probably a couple more that should be added    
    end
    
    properties(Access = private)
        DATA    % Stores data to send to the Robot
        % row vector [a3, a2, a1, a0, ts, tf], stores coefficients start
        % time and end time for each polynomial. 
        % rows: Pieces
        % pages: Coordinates
    end
        
    

    methods 
        %     Constructor
        function obj = robot_trj(x, t)
                        
            obj = obj@taskTrajectory(x, t, robot_trj.TS, robot_trj.DOF);
            
            obj.DATA = obj.robotDATA;
            
        end
        
        %Accessors
        % returns trajectories for all degrees of freedom. 
        function DATA = getDATA(obj)
            DATA = obj.DATA;
        end
        
        
        % Returns data for one DOF
        function xDATA = getCoordinateDATA(obj, coordinate)
            xDATA = obj.DATA(:,:,coordinate);
        end
        
        % generates some decent plots. 
        % Sepperates end effector orientation/gripping from translation
        function plotTrajectories(obj)
            figure;
            subplot(2, 1, 1); 
            p1 = plot(obj.getTimeseries, obj.X(1:3, :));
            title("End Effector Position");
            xlabel("time(s)");
            ylabel("m");
            legend({"q1", "q2", "q3"});         

            subplot(2, 1, 2);
            p2 = plot(obj.getTimeseries, obj.X(4:5, :));
            title("Gripping and Orientation");
            xlabel("time(s)");
            ylabel("radians");
            legend("q4", "gripper");
        
        end
        
    end
    
    
    methods (Static)
        % takes an array of robot_trj 
        % returns combined data from all trajectories
        % n is number of trajectories
        function DATA = combineDATA(trajectories, n)
              DATA = [];
            % Combine Trajectories
            % I think this might be easier using cat?
            for i = 1:n
            DATA = cat(1, DATA, trajectories(i).getDATA);
            end
                    
        end
        
    end
    
    
    methods(Access = private)
        
        
        % Converts trajectory coefficients into the correct form to send to
        % The Great Jenghis Khan.

        function DATA = robotDATA(obj)
            
            DATA = zeros(obj.pieces, obj.DATACOLS, obj.DOF);
            
            TimeVals = obj.getTime;
            
            for x = 1:obj.DOF
                % Get Coefficients for one coordinate trajectory
                Coeffs = obj.getCoefficients;
                for p = 1:obj.pieces
                    % shrink data
                    for a = 1:4
                        DATA(p, a, x) = obj.myShrink(Coeffs(p, a, x));
                    end
                    DATA(p, 5:6, x) = TimeVals(p:p + 1);
                end
                
            end
            
        end        
        
        % Truncates values for sending to controller
        function val = myShrink(obj, x)
            if abs(x) < obj.TruncationK
                val = 0;
            else
                val = x;
            end
        end
        
        
    end
    
    
end

    

